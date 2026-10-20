$ErrorActionPreference = 'Stop'
$taskRoot=Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$taskOut=Join-Path $PSScriptRoot 'host_tests\as5600'
New-Item -ItemType Directory -Path $taskOut -Force | Out-Null
$taskSource=[IO.File]::ReadAllText((Join-Path $taskRoot 'Core\Src\as5600.c'))
$taskStart=$taskSource.IndexOf('static HAL_StatusTypeDef as5600_read_regs(')
$taskEnd=$taskSource.IndexOf('HAL_StatusTypeDef AS5600_ReadSample(', $taskStart)
$taskPrefix=@'
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
typedef struct { uint32_t ErrorCode; } MockI2c;
static MockI2c hi2c1;
static uint8_t s_ready=1U;
static uint32_t s_last_error=99U;
volatile uint32_t g_as5600_busy_skips;
static uint32_t busy,reads,initializations,handled,mock_wait_ms;
static HAL_StatusTypeDef result=HAL_OK,handled_status;
#define RESET 0U
#define I2C_FLAG_BUSY 1U
#define HAL_I2C_ERROR_NONE 0U
#define AS5600_I2C_ADDRESS_7BIT 0x36U
#define I2C_MEMADD_SIZE_8BIT 1U
#define AS5600_I2C_TIMEOUT_MS 4U
#define AS5600_ERR_NONE 0U
#define __HAL_I2C_GET_FLAG(h,f) ((void)(h),(void)(f),busy)
static void AS5600_BusInit(void) { initializations++; s_ready=1U; }
static void handle_error(HAL_StatusTypeDef status) { handled++; handled_status=status; }
static HAL_StatusTypeDef HAL_I2C_Mem_Read(MockI2c *h,uint16_t address,uint8_t reg,uint32_t size,uint8_t *buf,uint16_t length,uint32_t timeout) {
  (void)h;(void)reg;(void)buf;(void)length;
  assert(address==0x6cU && size==1U && timeout==4U);
  reads++; if(busy) { mock_wait_ms+=26U; } return result;
}
'@
$taskTests=@'
int main(void) {
  uint8_t bytes[3]={0};
  busy=1U; hi2c1.ErrorCode=4U;
  assert(as5600_read_regs(0x0bU,bytes,3U)==HAL_BUSY);
  assert(reads==0U && mock_wait_ms==0U && g_as5600_busy_skips==1U);
  assert(handled==1U && handled_status==HAL_BUSY && hi2c1.ErrorCode==0U);
  busy=0U; assert(as5600_read_regs(0x0bU,bytes,3U)==HAL_OK);
  assert(reads==1U && s_last_error==AS5600_ERR_NONE && handled==1U);
  result=HAL_TIMEOUT; assert(as5600_read_regs(0x0bU,bytes,3U)==HAL_TIMEOUT);
  assert(reads==2U && handled==2U && handled_status==HAL_TIMEOUT);
  s_ready=0U; busy=1U;
  assert(as5600_read_regs(0x0bU,bytes,3U)==HAL_BUSY);
  assert(initializations==1U && reads==2U && g_as5600_busy_skips==2U);
  printf("PASS: BUSY never enters HAL's fixed wait, stale AF is cleared, initialization and normal/error paths retained.\n");
  return 0;
}
'@
[IO.File]::WriteAllText((Join-Path $taskOut 'as5600_busy_test.c'),($taskPrefix+"`r`n"+$taskSource.Substring($taskStart,$taskEnd-$taskStart)+$taskTests),[Text.UTF8Encoding]::new($false))
$taskVcvars=& 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -utf8 -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -find VC\Auxiliary\Build\vcvars64.bat
if(!$taskVcvars) { throw 'MSVC unavailable' }
$taskEnvironmentLines=& cmd.exe /d /c ('call "'+$taskVcvars+'" >nul && set')
if($LASTEXITCODE -ne 0) { throw 'MSVC environment setup failed' }
foreach($taskLine in $taskEnvironmentLines) { $taskPos=$taskLine.IndexOf('='); if($taskPos -gt 0) { [Environment]::SetEnvironmentVariable($taskLine.Substring(0,$taskPos),$taskLine.Substring($taskPos+1),'Process') } }
& cl.exe /nologo /W3 /WX /utf-8 /TC /std:c11 (Join-Path $taskOut 'as5600_busy_test.c') ('/Fo:'+(Join-Path $taskOut 'as5600_busy_test.obj')) ('/Fe:'+(Join-Path $taskOut 'as5600_busy_test.exe'))
if($LASTEXITCODE -ne 0) { throw 'AS5600 BUSY test build failed' }
& (Join-Path $taskOut 'as5600_busy_test.exe')
if($LASTEXITCODE -ne 0) { throw 'AS5600 BUSY regression failed' }
