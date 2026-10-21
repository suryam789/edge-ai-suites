using Windows.ApplicationModel.ConversationalAgent;

namespace AtaWakeAgent;

/// <summary>
/// The hardware wake-word logic. Two responsibilities:
///   1. Arm()  - enable a DSP-provisioned keyword so Windows wakes this agent.
///   2. OnWakeAsync() - run the 2-stage verify, then trigger ATA.
///
/// NOTE: the WinRT activation API surface (ActivationSignalDetector*) varies
/// slightly across Windows SDK versions. The calls below follow the official
/// "Voice Activation / ConversationalAgent" sample; validate member names
/// against the SDK you build with. This file compiles against WinRT but only
/// does anything real on an Intel DSP device with a provisioned keyword.
/// </summary>
internal static class WakeAgent
{
    // Cortana (en-US) identifiers read from the device's AarConfig at:
    //   HKCU\SOFTWARE\Microsoft\Speech_OneCore\Settings\VoiceActivation\
    //     <PackageFamilyName>!App\
    //       SignalDetectionConfigurations\{SignalId}\1033\{ModelId}  (DisplayName=Cortana)
    // SignalId = Intel HW keyword detector; ModelId = Cortana keyword model; 1033 = en-US.
    // Prefer discovering these at runtime via GetAllActivationSignalDetectors +
    // GetSupportedModelIdsForSignalId; these constants are the known-good fallback.
    private const string SignalId = "787E8510-069C-456F-AECE-068D884999EE";
    private const string ModelId = "86FA368E-357C-408C-BFCF-B89CBA2B0006";
    private const string DisplayName = "Cortana";

    // ConversationalAgentDetectorManager is a Windows Limited Access Feature; Windows
    // denies access unless the feature is unlocked with a Microsoft-issued token bound
    // to this package's identity.
    private const string ConversationalAgentFeatureId =
        "com.microsoft.windows.applicationmodel.conversationalagent_v1";

    /// <summary>
    /// Registers + enables the keyword configuration so the always-on DSP
    /// detector will wake this agent. Idempotent.
    /// </summary>
    public static async Task ArmAsync()
    {
        TryUnlockConversationalAgentFeature();
        var manager = ConversationalAgentDetectorManager.Default;
        var detectors = manager.GetAllActivationSignalDetectors().ToList();
        AgentLog.Write($"Windows returned {detectors.Count} activation signal detector(s).");
        foreach (var candidate in detectors)
            AgentLog.Write($"Detector provider={candidate.ProviderId}; configurable={candidate.CanCreateConfigurations}.");

        var detector = detectors.FirstOrDefault(candidate => candidate.CanCreateConfigurations);

        if (detector is null)
            throw new InvalidOperationException(
                "No activation signal detector found. This device has no Intel DSP keyword detector.");

        // Create the configuration if it does not exist yet, then enable it.
        var existing = detector.GetConfigurations();
        AgentLog.Write($"Selected detector '{detector.ProviderId}' has {existing.Count} existing configuration(s).");
        if (!existing.Any(c => c.SignalId == SignalId))
        {
            AgentLog.Write($"Creating signal configuration {SignalId}/{ModelId}.");
            await detector.CreateConfigurationAsync(SignalId, ModelId, DisplayName);
        }

        var config = detector.GetConfigurations().First(c => c.SignalId == SignalId);
        await config.SetEnabledAsync(true);

        AgentLog.Write($"Armed keyword '{SignalId}' on detector '{detector.ProviderId}'.");
    }

    // Unlocks the conversational-agent Limited Access Feature. The token/attestation are
    // issued by Microsoft for this package identity and supplied via environment so no code
    // change is needed once obtained. Without them Windows denies ConversationalAgent access.
    private static void TryUnlockConversationalAgentFeature()
    {
        var token = Environment.GetEnvironmentVariable("ATA_WAKE_LAF_TOKEN");
        var attestation = Environment.GetEnvironmentVariable("ATA_WAKE_LAF_ATTESTATION");
        if (string.IsNullOrWhiteSpace(token) || string.IsNullOrWhiteSpace(attestation))
        {
            AgentLog.Write(
                "ConversationalAgent is a Limited Access Feature and is locked. Set ATA_WAKE_LAF_TOKEN " +
                "and ATA_WAKE_LAF_ATTESTATION (issued by Microsoft for this package identity) to arm the DSP keyword.");
            return;
        }
        try
        {
            var result = Windows.ApplicationModel.LimitedAccessFeatures.TryUnlockFeature(
                ConversationalAgentFeatureId, token, attestation);
            AgentLog.Write($"LimitedAccessFeatures unlock status for conversationalagent: {result.Status}.");
        }
        catch (Exception ex)
        {
            AgentLog.Write($"LimitedAccessFeatures unlock threw: {ex.Message}");
        }
    }

    /// <summary>
    /// Invoked by the UWP activation host after Windows reports a DSP wake.
    /// Confirms the keyword, then triggers ATA.
    /// </summary>
    public static async Task OnWakeAsync()
    {
        // Best-effort: when voice-activated, Windows gives us a session to drive
        // the agent state. When launched any other way (or if a full-trust app
        // does not receive a session), we still trigger ATA - launch == wake.
        ConversationalAgentSession? session = null;
        try
        {
            session = await ConversationalAgentSession.GetCurrentSessionAsync();
        }
        catch (Exception ex)
        {
            AgentLog.Write($"No ConversationalAgentSession ({ex.Message}); triggering ATA anyway.");
        }

        if (session is not null)
        {
            if (!await Verify2ndStageAsync(session))
            {
                AgentLog.Write("2nd-stage verify rejected the wake (false alarm). Sleeping.");
                return;
            }
            try { await session.RequestAgentStateChangeAsync(ConversationalAgentState.Listening); } catch { }
        }

        var ok = await AtaBridge.TriggerAsync();
        AgentLog.Write(ok ? "ATA session started." : "ATA trigger failed.");

        if (session is not null)
        {
            try { await session.RequestAgentStateChangeAsync(ConversationalAgentState.Inactive); } catch { }
        }
    }

    /// <summary>
    /// Second-stage (software) keyword confirmation over the DSP-buffered audio.
    /// The hardware 1st stage is deliberately trigger-happy; this rejects the
    /// false positives. STUB: always accepts so the pipeline is testable before
    /// a real verifier/model is wired in.
    /// </summary>
    private static Task<bool> Verify2ndStageAsync(ConversationalAgentSession session)
    {
        // TODO: read session audio and run a small keyword verifier (e.g. an
        // ONNX model) before returning true. Kept as a stub for now.
        _ = session;
        return Task.FromResult(true);
    }
}

