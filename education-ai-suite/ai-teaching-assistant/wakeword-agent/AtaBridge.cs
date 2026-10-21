using System.Text;

namespace AtaWakeAgent;

/// <summary>
/// Hands a detected wake off to the existing ATA pipeline. This is the only
/// integration point with the rest of the app: a single POST to kiosk-core's
/// session endpoint - the same thing a mic-button press or the software
/// openWakeWord path does. No audio/ASR/RAG/TTS logic lives here.
/// </summary>
internal static class AtaBridge
{
    private static readonly HttpClient Http = new() { Timeout = TimeSpan.FromSeconds(15) };

    /// <summary>
    /// Tells kiosk-core to open a normal voice session now. kiosk-core then
    /// captures the mic and runs ASR -> RAG -> LLM -> TTS exactly as today.
    /// </summary>
    public static async Task<bool> TriggerAsync()
    {
        var baseUrl = Environment.GetEnvironmentVariable("KIOSK_CORE_UI_BASE_URL")
                      ?? "http://127.0.0.1:8012";

        // Mirrors the payload mic_session.py sends on /sessions/start.
        var payload = """
        {
          "sample_rate": 16000,
          "chunk_seconds": 4.0,
          "silence_timeout_seconds": 1.5,
          "max_session_seconds": 20.0,
          "language": "en",
          "tts_model": "kokoro",
          "tts_language": "English"
        }
        """;

        using var body = new StringContent(payload, Encoding.UTF8, "application/json");
        try
        {
            var resp = await Http.PostAsync($"{baseUrl}/api/v1/sessions/start", body);
            Console.WriteLine($"[AtaBridge] kiosk-core -> {(int)resp.StatusCode} {resp.StatusCode}");
            return resp.IsSuccessStatusCode;
        }
        catch (Exception ex)
        {
            Console.WriteLine($"[AtaBridge] Failed to reach kiosk-core at {baseUrl}: {ex.Message}");
            Console.WriteLine("[AtaBridge] Is start_ata.ps1 running?");
            return false;
        }
    }
}
