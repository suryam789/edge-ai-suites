using System;
using System.Threading.Tasks;
using Windows.ApplicationModel.Activation;
using Windows.ApplicationModel.ConversationalAgent;
using Windows.ApplicationModel;
using Windows.Storage;
using Windows.UI.Xaml;

namespace AtaWakeAgent.ActivationHost
{
sealed partial class App : Application
{
    public App()
    {
        InitializeComponent();
    }

    protected override async void OnLaunched(LaunchActivatedEventArgs args)
    {
        var session = await GetCurrentSessionAsync();
        var group = session == null ? "Arm" : "Wake";
        await LaunchWorkerAsync(group);
        // Headless launcher: close so no empty window lingers (worker runs independently).
        Exit();
    }

    protected override async void OnActivated(IActivatedEventArgs args)
    {
        base.OnActivated(args);
        var session = await GetCurrentSessionAsync();
        if (session != null)
            await LaunchWorkerAsync("Wake");
        Exit();
    }

    private static async Task<ConversationalAgentSession> GetCurrentSessionAsync()
    {
        try
        {
            return await ConversationalAgentSession.GetCurrentSessionAsync();
        }
        catch (Exception ex)
        {
            await LogAsync($"Unable to get conversational-agent session: {ex}");
            return null;
        }
    }

    private static async Task LaunchWorkerAsync(string parameterGroup)
    {
        try
        {
            await LogAsync($"Launching full-trust worker with parameter group '{parameterGroup}'.");
            await FullTrustProcessLauncher.LaunchFullTrustProcessForCurrentAppAsync(parameterGroup);
            await LogAsync($"Worker launch request accepted for '{parameterGroup}'.");
        }
        catch (Exception ex)
        {
            await LogAsync($"Wake worker launch failed: {ex}");
        }
    }

    private static async Task LogAsync(string message)
    {
        try
        {
            var file = await ApplicationData.Current.LocalFolder.CreateFileAsync(
                "wake-host.log", CreationCollisionOption.OpenIfExists);
            await FileIO.AppendTextAsync(file, $"{DateTimeOffset.Now:O} {message}{Environment.NewLine}");
        }
        catch (Exception ex)
        {
            System.Diagnostics.Debug.WriteLine($"Wake host logging failed: {ex}");
        }
    }
}
}