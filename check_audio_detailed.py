import sounddevice as sd

print("--- Details of audio devices ---")
try:
    devices = sd.query_devices()
    for i, device in enumerate(devices):
        # Vamos focar apenas nos dispositivos que têm canais de entrada (microfones)
        if device['max_input_channels'] > 0:
            print(f"\nInput device #{i}: {device['name']}")
            print(f"  Input channels: {device['max_input_channels']}")
            print(f"  Sample rate: {device['default_samplerate']} Hz")
except Exception as e:
    print(f"Error: {e}")

print("\n--- End ---")