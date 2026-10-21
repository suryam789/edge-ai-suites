# ATA Wake Agent (hardware wake-word)

A native wake agent intended to let the **Intel always-on DSP** wake the AI
Teaching Assistant without launching MvaApplicationOne. A UWP activation host
receives the Windows voice activation and dispatches the existing full-trust
.NET worker, which starts the normal ATA session.

## What it does

```
Mic -> Intel DSP (provisioned keyword) -> Windows AarSvc / ConversationalAgent
    -> this agent (verify) -> POST /api/v1/sessions/start -> kiosk-core (existing pipeline)
```

The agent does **not** do ASR/RAG/LLM/TTS. It only gets woken, confirms the
keyword, and triggers a normal ATA session - identical to pressing the mic
button or the software openWakeWord path. The whole Python pipeline is unchanged.

## Files

| File | Role |
|---|---|
| `Program.cs` | CLI for setup/testing (`--arm`, `--wake`, `--trigger`) |
| `WakeAgent.cs` | Arms the DSP keyword; handles the wake; 2nd-stage verify (stub) |
| `AtaBridge.cs` | The single HTTP call to kiosk-core (fully working today) |
| `Package.appxmanifest` | MSIX manifest with personal-assistant activation and full-trust worker declarations |
| `activation-host` | UWP app lifecycle entrypoint and activation-to-worker handoff |
| `AtaWakeAgent.csproj` | .NET 8 full-trust worker and ATA HTTP bridge |

## Build

Requires the **.NET 8 SDK**, **Visual Studio 2022 with the Universal Windows
Platform development workload**, and the **Windows 10/11 SDK (19041+)**.

```powershell
cd wakeword-agent
.\package.ps1 -Install
```

Run `package.ps1 -Install` from an elevated PowerShell. It builds the UWP host
and .NET worker, signs and installs the MSIX, updates the root `.env` with the
installed AUMID, and launches ATA's host once to arm its DSP keyword.

## Test the bridge now (no hardware needed)

With ATA running (`start_ata.ps1`), prove the hand-off end-to-end:

```powershell
dotnet run -- --trigger
```

A successful call starts a voice session in the ATA UI. This validates
everything except the DSP detection itself.

## The one external dependency

The DSP can only listen for a keyword whose **acoustic model is provisioned
into the DSP firmware** (shipped via the Intel MVA driver extension). You can:

- **Prototype** by pointing `SignalId`/`ModelId` in `WakeAgent.cs` at a keyword
  already provisioned on the device; or
- Get a **custom** "Hey Teaching Assistant" model provisioned by Intel/OEM.

The provisioned DSP keyword model is a device/driver prerequisite. ATA's own
package does not require or launch MvaApplicationOne.

## Known stub

`Verify2ndStageAsync` in `WakeAgent.cs` currently always accepts. Replace it
with a real software keyword verifier over the session audio before shipping.
