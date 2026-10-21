# =============================================================================
# register-keyword.ps1  -  "mini-MvaHelper" for the ATA Wake Agent.
#
# Registers OUR installed agent for a hardware keyword (default Cortana/en-US)
# by writing the exact voice-activation entries reverse-engineered from a
# working MVA machine. This replaces MvaHelper's essential job so ATA does not
# depend on MvaApplicationOne.
#
# Writes under HKCU (per-user) - no admin needed for this step. The agent MSIX
# must already be installed (so its PackageFamilyName exists).
#
#   .\register-keyword.ps1              # register Cortana/en-US
#   .\register-keyword.ps1 -Unregister  # remove the registration
# =============================================================================
param(
    [string]$PackageName = "Intel.ATA.WakeAgent",
    [string]$AppId       = "AtaWakeAgent",
    [string]$Keyword     = "Cortana",
    [int]   $Langid      = 0x409,                                    # en-US
    [string]$SignalId    = "{787E8510-069C-456F-AECE-068D884999EE}", # Intel HW detector
    [string]$ModelId     = "{86FA368E-357C-408C-BFCF-B89CBA2B0006}", # Cortana model
    [switch]$Unregister  = $false
)

$ErrorActionPreference = "Stop"

$pkg = Get-AppxPackage -Name $PackageName -ErrorAction SilentlyContinue
if (-not $pkg) {
    Write-Host "[!] Package '$PackageName' is not installed. Install the MSIX first (package.ps1 -Install)." -ForegroundColor Red
    exit 1
}
$appKey = "$($pkg.PackageFamilyName)!$AppId"      # e.g. Intel.ATA.WakeAgent_xxxx!AtaWakeAgent
Write-Host "[*] Target app: $appKey" -ForegroundColor Cyan

$root    = "HKCU:\SOFTWARE\Microsoft\Speech_OneCore\Settings\VoiceActivation"
$appPath = Join-Path $root $appKey
$locale  = [string]$Langid                         # 1033 for en-US
$cfgPath = Join-Path $root "SignalDetectionConfigurations\$appKey\$SignalId\$locale\$ModelId"

if ($Unregister) {
    Remove-Item (Join-Path $root "SignalDetectionConfigurations\$appKey") -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item $appPath -Recurse -Force -ErrorAction SilentlyContinue
    Write-Host "[OK] Unregistered $appKey" -ForegroundColor Green
    return
}

# Top-level flag: use the AAR config mapping.
New-Item -Path $root -Force | Out-Null
New-ItemProperty -Path $root -Name "UseAarConfigFile" -Value 1 -PropertyType DWord -Force | Out-Null

# Per-app voice-activation entry (matches MvaApplicationOne's keys).
New-Item -Path $appPath -Force | Out-Null
New-ItemProperty -Path $appPath -Name "LANGID" -Value $Langid -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $appPath -Name "IsLanguageAvailable" -Value 1 -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $appPath -Name "AgentActivationEnabled" -Value 1 -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $appPath -Name "AgentActivationOnLockScreenEnabled" -Value 1 -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $appPath -Name "DisplayName" -Value $Keyword -PropertyType String -Force | Out-Null
New-ItemProperty -Path $appPath -Name "HwKeywordState" -Value 4 -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $appPath -Name "SwKeywordState" -Value 1 -PropertyType DWord -Force | Out-Null

# Keyword/signal binding: <app>\{SignalId}\<locale>\{ModelId}.
New-Item -Path $cfgPath -Force | Out-Null
New-ItemProperty -Path $cfgPath -Name "Available" -Value 1 -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $cfgPath -Name "Active" -Value 1 -PropertyType DWord -Force | Out-Null
New-ItemProperty -Path $cfgPath -Name "DisplayName" -Value $Keyword -PropertyType String -Force | Out-Null

Write-Host "[OK] Registered '$Keyword' (LANGID $Langid) for $appKey" -ForegroundColor Green
Write-Host "    Say '$Keyword' to wake the agent. If nothing happens, the activation" -ForegroundColor Yellow
Write-Host "    may require a UWP-style app - see wakeword-agent/README.md." -ForegroundColor Yellow
