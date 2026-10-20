$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$taskDir = Join-Path $PSScriptRoot 'host_tests\encoder_filter'
New-Item -ItemType Directory -Force -Path $taskDir | Out-Null
$taskSource = [IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\app_vehicle.c'))
$taskWrapper = [regex]::Match($taskSource,'(?s)static void s_enc_filter_flush_ready\(void\).*?\r?\n}')
if (!$taskWrapper.Success) { throw 'Production encoder flush wrapper missing' }
$taskSpeedWrapper = [regex]::Match($taskSource,'(?s)static float s_enc_speed_read\(.*?\r?\n}')
if (!$taskSpeedWrapper.Success) { throw 'Production hybrid speed wrapper missing' }
$taskPrefix = @'
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <math.h>
#define APP_WHEEL_COUNT 4U
volatile int32_t g_enc_count[4];
volatile int8_t g_enc_sign[4];
#include "app_encoder_filter.inc"
typedef struct { volatile uint32_t IDR; } TestPort;
typedef struct { volatile uint32_t PR; } TestExti;
typedef struct { volatile uint32_t CYCCNT; } TestDwt;
static TestPort test_a,test_b;
static TestExti test_exti;
static TestDwt test_dwt;
static int critical;
static uint32_t SystemCoreClock=1000000U;
#define GPIOA (&test_a)
#define GPIOB (&test_b)
#define EXTI (&test_exti)
#define DWT (&test_dwt)
#define GPIO_PIN_0 (1U<<0)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_6 (1U<<6)
#define GPIO_PIN_12 (1U<<12)
#define EXTI_PR_PR0 GPIO_PIN_0
#define EXTI_PR_PR4 GPIO_PIN_4
#define EXTI_PR_PR6 GPIO_PIN_6
#define EXTI_PR_PR12 GPIO_PIN_12
#define taskENTER_CRITICAL() do { assert(!critical); critical=1; } while(0)
#define taskEXIT_CRITICAL() do { assert(critical); critical=0; } while(0)
'@
$taskTests = @'
static void reset_test(void) {
  uint8_t i,levels[4]={0,0,0,0};
  for(i=0;i<4;i++) {g_enc_count[i]=0;g_enc_sign[i]=1;}
  s_enc_filter_init(levels,100U); /* Test clock: one cycle per microsecond. */
  test_a.IDR=0;test_b.IDR=0;test_exti.PR=0;test_dwt.CYCCNT=0;
}
/* Exact periodic edge input at staggered wheel phases, sampled by the same
 * production qualifier + speed wrapper at 5ms. This tests measurement only. */
static void low_speed_test(uint32_t period, float wanted)
{
  uint32_t t,w,next[4],edge[4]={0},last[4]={0},samples=0;
  float filtered[4]={0},mean=0,median=0;
  reset_test();
  for(w=0;w<4;w++) next[w]=1000U+w*period/4U;
  for(t=5000U;t<=5000000U;t+=5000U)
  {
    float sum=0,sorted[4];
    for(w=0;w<4;w++)
    {
      uint32_t j;
      while(next[w]<=t)
      {
        uint8_t a=(uint8_t)((++edge[w])&1U);
        s_enc_filter_edge((uint8_t)w,a,(uint8_t)!a,next[w]);
        next[w]+=period;
      }
      s_enc_filter_flush((uint8_t)w,(uint8_t)(edge[w]&1U),t);
      test_dwt.CYCCNT=t;
      {
        float count_rate=(float)((uint32_t)g_enc_count[w]-last[w])/0.005f;
        float rate=s_enc_speed_read((uint8_t)w,count_rate);
        last[w]=(uint32_t)g_enc_count[w];
        filtered[w]+=0.2f*(rate*60.0f/468.6f-filtered[w]);
      }
      assert(isfinite(filtered[w]) && !critical);
      sum+=filtered[w];sorted[w]=filtered[w];
      for(j=w;j>0 && sorted[j-1]>sorted[j];j--)
      { float tmp=sorted[j-1];sorted[j-1]=sorted[j];sorted[j]=tmp; }
    }
    if(t>1000000U) {mean+=sum/4;median+=(sorted[1]+sorted[2])/2;samples++;}
  }
  mean/=(float)samples;median/=(float)samples;
  assert(fabsf(mean-wanted)<wanted*0.03f && fabsf(median-wanted)<wanted*0.03f);
  printf("PASS: staggered %.2fRPM edges: estimator mean %.3f, middle-pair %.3fRPM\n",wanted,mean,median);
  for(t=5005000U;t<=5500000U;t+=5000U)
    for(w=0;w<4;w++)
    {
      test_dwt.CYCCNT=t;
      filtered[w]+=0.2f*(s_enc_speed_read((uint8_t)w,0)*60/468.6f-filtered[w]);
    }
  for(w=0;w<4;w++) assert(fabsf(filtered[w])<0.001f && !s_enc_speed_direction[w]);
}
int main(void) {
  uint32_t t,i; uint8_t w;
  reset_test();
  for(i=0;i<1000;i++) s_enc_filter_edge(3,0,(uint8_t)(i&1U),i);
  assert(g_enc_irq[3]==1000 && g_enc_duplicate[3]==1000 && !g_enc_count[3]);
  reset_test();
  for(i=0;i<500;i++) {
    s_enc_filter_edge(3,1,0,i*50U);
    s_enc_filter_edge(3,0,0,i*50U+20U);
  }
  s_enc_filter_flush(3,0,30000U);
  assert(g_enc_glitch[3]==500 && !g_enc_accepted[3] && !g_enc_count[3]);
  reset_test();
  s_enc_filter_edge(3,1,0,0);
  s_enc_filter_flush(3,1,99); assert(!g_enc_count[3]);
  s_enc_filter_edge(3,1,1,100); assert(g_enc_count[3]==1);
  for(i=101;i<500;i++) s_enc_filter_edge(3,1,0,i);
  assert(g_enc_count[3]==1 && g_enc_accepted[3]==1);
  s_enc_filter_edge(3,0,1,500);
  s_enc_filter_flush(3,0,600); assert(g_enc_count[3]==2);
  /* Every channel at 457us/edge (~280 RPM), both direction signs. */
  for(w=0;w<4;w++) {
    reset_test(); g_enc_sign[w]=(w&1U)?-1:1;
    for(i=0;i<200;i++) {
      uint8_t a=(uint8_t)((i+1U)&1U);
      t=i*457U;
      s_enc_filter_edge(w,a,(uint8_t)!a,t);
      s_enc_filter_flush(w,a,t+100U);
    }
    assert(g_enc_count[w]==200*g_enc_sign[w] && g_enc_accepted[w]==200);
    assert(!g_enc_glitch[w]);
    for(i=0;i<200;i++) {
      uint8_t a=(uint8_t)((i+1U)&1U);
      t=100000U+i*457U;
      s_enc_filter_edge(w,a,a,t); s_enc_filter_flush(w,a,t+100U);
    }
    assert(!g_enc_count[w] && g_enc_accepted[w]==400);
  }
  /* Continuous edges commit the prior edge without requiring task service. */
  reset_test();
  s_enc_filter_edge(0,1,0,0); s_enc_filter_edge(0,0,1,457);
  s_enc_filter_edge(0,1,0,914); assert(g_enc_count[0]==2);
  s_enc_filter_flush(0,1,1014); assert(g_enc_count[0]==3);
  /* Production wrapper must not discard a real fall waiting in EXTI PR. */
  reset_test(); s_enc_filter_edge(0,1,0,0);
  test_exti.PR=EXTI_PR_PR0; test_dwt.CYCCNT=457;
  s_enc_filter_flush_ready(); assert(!g_enc_count[0] && !g_enc_glitch[0] && !critical);
  s_enc_filter_edge(0,0,1,457); test_exti.PR=0; test_dwt.CYCCNT=557;
  s_enc_filter_flush_ready(); assert(g_enc_count[0]==2 && !critical);
  /* All physical A pins reach the corresponding raw-encoder slot. */
  reset_test();
  for(w=0;w<4;w++) s_enc_filter_edge(w,1,0,0);
  test_a.IDR=GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_6;test_b.IDR=GPIO_PIN_12;
  test_dwt.CYCCNT=100;s_enc_filter_flush_ready();
  for(w=0;w<4;w++) assert(g_enc_count[w]==1);
  /* An out-and-back state missed by the IRQ never manufactures a count. */
  reset_test();s_enc_filter_edge(0,1,0,0);s_enc_filter_flush(0,0,200);
  assert(!g_enc_count[0] && g_enc_glitch[0]==1);
  reset_test();s_enc_filter_edge(0,1,0,UINT32_MAX-50U);
  s_enc_filter_flush(0,1,49U);assert(g_enc_count[0]==1);
  /* Direction changes need two new same-direction accepted edges. */
  reset_test();
  s_enc_filter_edge(0,1,0,1000);s_enc_filter_flush(0,1,1100);
  test_dwt.CYCCNT=1100;assert(s_enc_speed_read(0,200)==200);
  s_enc_filter_edge(0,0,1,129041);s_enc_filter_flush(0,0,129141);
  test_dwt.CYCCNT=129141;
  assert(fabsf(s_enc_speed_read(0,0)*60/468.6f-1)<0.0001f);
  test_dwt.CYCCNT=329041;assert(s_enc_speed_read(0,0)<7.81f);
  test_dwt.CYCCNT=379041;assert(s_enc_speed_read(0,0)==0);
  test_dwt.CYCCNT=129141;assert(s_enc_speed_read(0,0)==0); /* no stale revival */
  s_enc_filter_edge(0,1,1,400000);s_enc_filter_flush(0,1,400100);
  test_dwt.CYCCNT=400100;assert(s_enc_speed_read(0,-200)==-200);
  s_enc_filter_edge(0,0,0,528041);s_enc_filter_flush(0,0,528141);
  test_dwt.CYCCNT=528141;
  assert(fabsf(s_enc_speed_read(0,0)*60/468.6f+1)<0.0001f);
  /* Existing accepted period survives rejected short pulse pairs. */
  s_enc_filter_edge(0,1,0,530000);s_enc_filter_edge(0,0,0,530020);
  test_dwt.CYCCNT=530100;
  assert(s_enc_speed_direction[0]==-1 && s_enc_speed_period[0]==128041);
  assert(s_enc_speed_read(0,0)<0);
  reset_test();
  s_enc_filter_edge(0,1,0,UINT32_MAX-10000U);s_enc_filter_flush(0,1,UINT32_MAX-9900U);
  s_enc_filter_edge(0,0,1,118040U);s_enc_filter_flush(0,0,118140U);
  test_dwt.CYCCNT=118140U;
  assert(fabsf(s_enc_speed_read(0,0)*60/468.6f-1)<0.0001f);
  assert(s_enc_speed_hybrid(2000,500,100,1,1000000)==2000); /* high speed=count */
  assert(s_enc_speed_hybrid(9,10000,100,1,1000000)==100); /* low speed=period */
  assert(fabsf(s_enc_speed_hybrid(400,5000,100,1,1000000)-300)<0.001f);
  low_speed_test(128041,1.0f);low_speed_test(25608,5.0f);low_speed_test(12804,10.0f);
  puts("PASS: production 100us stable-level qualifier, duplicate IRQ burst, short pulse pairs, four-wheel signs, normal 280RPM edges, queued EXTI race and cycle wrap");
  puts("PASS: low-speed period/high-speed count blend, ageing/250ms stop, first edge/reversal, accepted-only timing, cycle wrap, and preserved sparse DC under median fusion");
  return 0;
}
'@
$taskC=$taskPrefix+"`r`n"+$taskWrapper.Value+"`r`n"+$taskSpeedWrapper.Value+"`r`n"+$taskTests
[IO.File]::WriteAllText((Join-Path $taskDir 'encoder_filter_test.c'),$taskC,[Text.UTF8Encoding]::new($false))
$taskVcvars=& 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -utf8 -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat
if(!$taskVcvars) { throw 'MSVC unavailable' }
$taskLines=& cmd.exe /d /c ('call "'+$taskVcvars+'" >nul && set')
foreach($taskLine in $taskLines) {
  $taskSep=$taskLine.IndexOf('='); if($taskSep -gt 0) {
    [Environment]::SetEnvironmentVariable($taskLine.Substring(0,$taskSep),$taskLine.Substring($taskSep+1),'Process')
  }
}
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 ('/I'+(Join-Path $taskRoot 'Core\Src')) (Join-Path $taskDir 'encoder_filter_test.c') ('/Fo:'+$taskDir+'\') ('/Fe:'+(Join-Path $taskDir 'encoder_filter_test.exe'))
if($LASTEXITCODE -ne 0) { throw 'Encoder host compile failed' }
& (Join-Path $taskDir 'encoder_filter_test.exe')
if($LASTEXITCODE -ne 0) { throw 'Encoder qualifier regression failed' }
