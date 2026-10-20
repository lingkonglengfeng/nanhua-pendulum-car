# Regression of actual production waveform builder, publication and buffer refill.
$ErrorActionPreference='Stop'
$taskRoot=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$taskTestDir=Join-Path $PSScriptRoot 'host_tests\motor_dma'
New-Item -ItemType Directory -Path $taskTestDir -Force | Out-Null
$taskSource=[IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\app_motor_dma.inc'))
$taskSource=$taskSource.Substring(0,$taskSource.IndexOf('/* MOTOR_WAVE_HOST_END */'))
$taskVehicle=[IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\app_vehicle.c'))
$taskSigns=[regex]::Match($taskVehicle,'volatile int8_t\s+g_mot_sign\[APP_WHEEL_COUNT\]\s*=\s*(\{[^}]+\});')
if (!$taskSigns.Success) { throw 'Production F103 motor signs missing' }
$taskApplyStart=$taskVehicle.IndexOf('static void s_motor_apply(uint8_t wheel, float u)')
$taskApplyEnd=$taskVehicle.IndexOf('static void s_motor_all_coast(void)',$taskApplyStart)
$taskApply=$taskVehicle.Substring($taskApplyStart,$taskApplyEnd-$taskApplyStart)
$taskPrefix=@'
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#define STM32F103xB 1
#define APP_MOTOR_PWM_SLOTS 64U
#define __DMB() ((void)0)
#define __get_PRIMASK() (0U)
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
volatile uint16_t g_bal_ccr[4];
volatile uint8_t g_bal_dir[4];
volatile float g_bal_u[4];
volatile int8_t g_mot_sign[4]=__PRODUCTION_MOTOR_SIGNS__;
'@
$taskPrefix=$taskPrefix.Replace('__PRODUCTION_MOTOR_SIGNS__',$taskSigns.Groups[1].Value)
$taskTests=@'
static uint32_t apply_bsrr(uint32_t state,uint32_t v) { return (state&~(v>>16))|(v&65535U); }
static void check_wave(uint8_t bank,const uint8_t *ticks,const uint8_t *reverse) {
 uint32_t slot,w,drive[4]={0},brake[4]={0};
 uint32_t a=0xA55AU,b=0x5AA5U;
 for(slot=0;slot<MOTOR_SLOTS;slot++) {
  uint32_t old_a=a,old_b=b,wa=s_motor_wave_a[bank][slot],wb=s_motor_wave_b[bank][slot];
  assert(((wa&65535U)&(wa>>16))==0 && ((wb&65535U)&(wb>>16))==0);
  a=apply_bsrr(a,wa); b=apply_bsrr(b,wb);
  assert(((a^old_a)&~MOTOR_GPIO_A_MASK)==0 && ((b^old_b)&~MOTOR_GPIO_B_MASK)==0);
  for(w=0;w<4;w++) {
   uint32_t in1=(a>>(8+w))&1U;
   uint32_t in2=(w<2)?((a>>(2+w))&1U):((b>>(12+w))&1U);
   if(ticks[w]==0) { assert(!in1&&!in2); }
   else if(in1&&in2) { brake[w]++; }
   else { assert(in1==(reverse[w]?0U:1U) && in2==(reverse[w]?1U:0U)); drive[w]++; }
  }
 }
 for(w=0;w<4;w++) { assert(drive[w]==ticks[w]); if(ticks[w]) { assert(brake[w]==MOTOR_SLOTS-ticks[w]); } }
}
int main(void) {
 uint32_t n,w,sum=0,old_late,gen;
 uint8_t ticks[4]={0},reverse[4]={0};
 for(n=0;n<=64;n++) {
  for(w=0;w<4;w++) { ticks[w]=(uint8_t)((n+w)%65); reverse[w]=(uint8_t)((n+w)&1U); }
  s_motor_wave_build(0,ticks,reverse); check_wave(0,ticks,reverse);
 }
 for(n=0;n<1000;n++) { sum+=s_motor_quantize(3,0.1f); }
 assert(fabsf((float)sum-6400.0f)<1.1f);
 assert(s_motor_quantize(3,0)==0 && s_motor_fraction[3]==0);
 assert(s_motor_quantize(3,1)==64 && s_motor_quantize(3,-1)==64);
 assert(s_motor_quantize(3,0.1f)==6 && s_motor_quantize(3,-0.1f)==6);
 for(w=0;w<4;w++) { s_motor_apply((uint8_t)w,0); }
 s_motor_apply(0,0.1f); s_motor_apply(3,0.1f); s_motor_dma_commit();
 assert(s_motor_requested[0]<0 && s_motor_requested[3]>0);
 assert(g_motor_on_ticks[0]==6 && g_motor_on_ticks[3]==6);
 assert(g_motor_in1_ticks[0]==58 && g_motor_in2_ticks[0]==64);
 assert(g_motor_in1_ticks[3]==64 && g_motor_in2_ticks[3]==58);
 assert(g_motor_on_ticks[1]==0 && g_motor_on_ticks[2]==0);
 assert(g_bal_dir[0]==1 && g_bal_dir[3]==0);
 gen=s_motor_wave_generation; assert(gen!=0);
 for(n=0;n<MOTOR_DMA_WORDS;n++) { s_motor_dma_a[n]=0x12345678U;s_motor_dma_b[n]=0x87654321U; }
 s_motor_refill(0,64,64); assert(s_motor_half_generation[0]==gen);
 for(n=0;n<64;n++) { assert(s_motor_dma_a[n]==s_motor_wave_a[s_motor_wave_bank][n]); assert(s_motor_dma_b[n+64]==0x87654321U); }
 old_late=g_motor_dma_late;
 s_motor_refill(1,64,64); assert(g_motor_dma_late==old_late+1 && s_motor_half_generation[1]!=gen);
 s_motor_refill(1,128,128); assert(s_motor_half_generation[1]==gen);
 for(n=0;n<64;n++) { assert(s_motor_dma_b[n+64]==s_motor_wave_b[s_motor_wave_bank][n]); }
 assert(!s_motor_half_safe(0,0,64) && !s_motor_half_safe(0,64,65));
 assert(!s_motor_half_safe(1,129,128) && !s_motor_half_safe(1,128,64));
 assert(!s_motor_half_safe(0,32,64) && !s_motor_half_safe(1,96,128));
 for(w=0;w<4;w++) { s_motor_apply((uint8_t)w,0); }
 s_motor_dma_commit();s_motor_refill(0,64,64);s_motor_refill(1,128,128);
 for(n=0;n<MOTOR_DMA_WORDS;n++) { assert(s_motor_dma_a[n]==MOTOR_GPIO_A_MASK<<16);assert(s_motor_dma_b[n]==MOTOR_GPIO_B_MASK<<16); }
 s_motor_apply(3,2.0f);s_motor_dma_commit();assert(g_motor_on_ticks[3]==64);
 s_motor_apply(1,0.1f);s_motor_apply(2,0.1f);s_motor_dma_commit();
 assert(s_motor_requested[1]<0 && s_motor_requested[2]>0);
 assert(g_bal_dir[1]==1 && g_bal_dir[2]==0);
 printf("PASS: mixed endpoint/intermediate waveforms use drive/brake in both directions; zero=coast; unrelated GPIO preserved.\n");
 printf("PASS: actual direction mapping and saturation, fractional duty, reversal, four-wheel publication and STOP coast.\n");
 printf("PASS: refill changes only the inactive half of BOTH streams; wrap/phase/deadline boundary cases reject writes.\n");
 return 0;
}
'@
$taskC=$taskPrefix+"`r`n"+$taskSource+"`r`n"+$taskApply+$taskTests
[IO.File]::WriteAllText((Join-Path $taskTestDir 'motor_dma_test.c'),$taskC,[Text.UTF8Encoding]::new($false))
$taskVswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$taskVcvars = & $taskVswhere -utf8 -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat
if (!$taskVcvars) { throw 'MSVC host compiler unavailable' }
$taskEnvironmentLines = & cmd.exe /d /c ('call "' + $taskVcvars + '" >nul && set')
if ($LASTEXITCODE -ne 0) { throw 'MSVC environment setup failed' }
foreach ($taskEnvironmentLine in $taskEnvironmentLines) {
    $taskSeparator = $taskEnvironmentLine.IndexOf('=')
    if ($taskSeparator -gt 0) {
        [Environment]::SetEnvironmentVariable($taskEnvironmentLine.Substring(0,$taskSeparator),$taskEnvironmentLine.Substring($taskSeparator+1),'Process')
    }
}
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 (Join-Path $taskTestDir 'motor_dma_test.c') ('/Fo:' + (Join-Path $taskTestDir 'motor_dma_test.obj')) ('/Fe:' + (Join-Path $taskTestDir 'motor_dma_test.exe'))
if ($LASTEXITCODE -ne 0) { throw 'Motor DMA host build failed' }
& (Join-Path $taskTestDir 'motor_dma_test.exe')
if ($LASTEXITCODE -ne 0) { throw 'Motor DMA regression failed' }
