# pip install librosa soundfile numpy pandas
# (recommended) install ffmpeg on your system for robust MP3 loading

# import numpy as np
# import pandas as pd
# import librosa
# import soundfile as sf
# from pathlib import Path
#
# # ========= EDIT THESE 4 LINES =========
# MP3_PATH   = r"C:\Users\ghoriss\Downloads\Rimsky.mp3"   # <- input MP3
# DURATION_S = 60.0                                              # seconds to analyze/export
# CSV_OUT    = r"C:\Users\ghoriss\Downloads\Rimsky_60s.csv"     # <- output CSV of beat times
# WAV_OUT    = r"C:\Users\ghoriss\Downloads\Rimsky_60s.wav"           # <- optional WAV clip (set '' to skip)
# # =====================================
#
# TARGET_SR = 44100  # resample target (Hz). 44.1k is fine.
#
# def analyze_first_minuteslice(mp3_path: str, duration_s: float, target_sr: int = TARGET_SR):
#     # Load only the first `duration_s` seconds as mono
#     y, sr = librosa.load(mp3_path, sr=target_sr, mono=True, duration=duration_s)
#     if y.size == 0:
#         raise RuntimeError("Audio not loaded. Check MP3_PATH and install ffmpeg if needed.")
#
#     # Beat tracking
#     tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr, trim=False)
#     beat_times = librosa.frames_to_time(beat_frames, sr=sr)
#
#     # Robust BPM via median inter-beat interval
#     if len(beat_times) > 1:
#         intervals = np.diff(beat_times)           # seconds
#         bpm_median = 60.0 / np.median(intervals)  # BPM
#     else:
#         intervals = np.array([])
#         bpm_median = float(tempo)
#
#     return {
#         "sr": sr,
#         "tempo_librosa_bpm": float(tempo),
#         "tempo_median_bpm": float(bpm_median),
#         "beat_times": beat_times,
#         "intervals": intervals,
#         "y": y,
#     }
#
# def main():
#     mp3 = Path(MP3_PATH)
#     if not mp3.exists():
#         raise FileNotFoundError(f"MP3 not found: {mp3}")
#
#     res = analyze_first_minuteslice(str(mp3), duration_s=DURATION_S)
#
#     print("\n=== BPM Analysis ===")
#     print(f"File: {mp3}")
#     print(f"Duration analyzed: {DURATION_S:.1f} s")
#     print(f"librosa tempo (BPM): {res['tempo_librosa_bpm']:.2f}")
#     print(f"median-interval BPM: {res['tempo_median_bpm']:.2f}")
#     print(f"beats found:         {len(res['beat_times'])}")
#     if len(res["beat_times"]) > 0:
#         print("first 8 beat times (s):", np.round(res["beat_times"][:8], 3))
#
#     # Save CSV
#     csv_path = Path(CSV_OUT) if CSV_OUT else mp3.with_name(mp3.stem + "_beats.csv")
#     csv_path.parent.mkdir(parents=True, exist_ok=True)
#     bt = pd.DataFrame({"beat_index": np.arange(len(res["beat_times"])),
#                        "time_sec": res["beat_times"]})
#     bt.to_csv(csv_path, index=False)
#     print(f"Saved beat timestamps -> {csv_path}")
#
#     # Optional WAV export (first DURATION_S seconds)
#     if WAV_OUT:
#         wav_path = Path(WAV_OUT)
#         wav_path.parent.mkdir(parents=True, exist_ok=True)
#         sf.write(str(wav_path), res["y"], res["sr"])
#         print(f"Saved {DURATION_S:.0f}s WAV clip -> {wav_path}")
#
# if __name__ == "__main__":
#     main()


# from pathlib import Path
# import pandas as pd
# import soundfile as sf
#
# WAV = Path(r"C:\Users\ghoriss\Downloads\Vivaldi_60s.wav")
# CSV = Path(r"C:\Users\ghoriss\Downloads\Vivaldi_beats_60s.csv")
#
# # Load audio length
# info = sf.info(str(WAV))
# wav_len = info.frames / info.samplerate
#
# # Load beats
# df = pd.read_csv(CSV)
# last_beat = float(df["time_sec"].iloc[-1])
#
# print(f"WAV length: {wav_len:.3f} s")
# print(f"Last beat:  {last_beat:.3f} s")
# if last_beat <= wav_len + 0.01:
#     print("✓ CSV beat times fit within WAV length. Looks aligned.")
# else:
#     print("⚠ Last beat exceeds WAV length. Regenerate CSV for this WAV.")


import numpy as np
import pandas as pd
import librosa
import soundfile as sf
from pathlib import Path

# ========= EDIT THESE LINES =========
MP3_PATH    = r"C:\Users\ghoriss\Downloads\Rimsky.mp3"       # input MP3
START_AT_S  = 75.0                                            # slice start (sec) -> 1:10
LENGTH_S    = 40.0                                            # slice length (sec) -> to 2:10
CSV_OUT     = r"C:\Users\ghoriss\Downloads\Rimsky_1m_slice_beats.csv"
WAV_OUT     = r"C:\Users\ghoriss\Downloads\Rimsky_1m_slice.wav"
TARGET_SR   = 44100                                           # resample target (Hz)
# =====================================

def analyze_slice(mp3_path: str, start_s: float, length_s: float, target_sr: int):
    """
    Loads [start_s, start_s + length_s] from MP3, mono, at target_sr.
    Runs beat tracking on the slice and returns audio + beat data.
    Beat times are relative to the start of the slice (i.e., first beat ~0s+).
    """
    # Load only the desired slice
    y, sr = librosa.load(mp3_path, sr=target_sr, mono=True, offset=start_s, duration=length_s)
    if y.size == 0:
        raise RuntimeError(
            "Audio slice not loaded. Check MP3_PATH and that ffmpeg/audioread can read this file."
        )

    # Beat tracking on the slice
    tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr, trim=False)
    beat_times = librosa.frames_to_time(beat_frames, sr=sr)  # seconds, relative to slice start

    # Robust BPM via median inter-beat interval
    if len(beat_times) > 1:
        intervals = np.diff(beat_times)           # seconds
        bpm_median = 60.0 / np.median(intervals)  # BPM
    else:
        intervals = np.array([])
        bpm_median = float(tempo)

    return {
        "sr": sr,
        "tempo_librosa_bpm": float(tempo),
        "tempo_median_bpm": float(bpm_median),
        "beat_times": beat_times,
        "intervals": intervals,
        "y": y,
        "slice_len": len(y) / sr,
    }

def main():
    mp3 = Path(MP3_PATH)
    if not mp3.exists():
        raise FileNotFoundError(f"MP3 not found: {mp3}")

    # Optional: sanity print of total duration
    try:
        total_dur = librosa.get_duration(path=str(mp3))
        print(f"Full file duration: {total_dur:.2f} s")
        if START_AT_S >= total_dur:
            raise ValueError(f"START_AT_S ({START_AT_S}) is beyond file length ({total_dur:.2f}s).")
        if START_AT_S + LENGTH_S > total_dur:
            print(f"Note: slice end ({START_AT_S + LENGTH_S:.2f}s) exceeds file length; "
                  f"we'll get a shorter slice.")
    except Exception:
        # If get_duration fails (e.g. missing backend), we can still proceed
        pass

    res = analyze_slice(str(mp3), START_AT_S, LENGTH_S, TARGET_SR)

    print("\n=== BPM Analysis (Slice) ===")
    print(f"File: {mp3}")
    print(f"Slice: start={START_AT_S:.2f}s, length requested={LENGTH_S:.2f}s, loaded={res['slice_len']:.2f}s")
    print(f"librosa tempo (BPM): {res['tempo_librosa_bpm']:.2f}")
    print(f"median-interval BPM: {res['tempo_median_bpm']:.2f}")
    print(f"beats found:         {len(res['beat_times'])}")
    if len(res["beat_times"]) > 0:
        print("first 8 beat times (s, relative to slice start):",
              np.round(res["beat_times"][:8], 3))

    # Save CSV (beat times relative to the start of this slice)
    csv_path = Path(CSV_OUT) if CSV_OUT else mp3.with_name(mp3.stem + "_slice_beats.csv")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    bt = pd.DataFrame({
        "beat_index": np.arange(len(res["beat_times"])),
        "time_sec": res["beat_times"]
    })
    bt.to_csv(csv_path, index=False)
    print(f"Saved beat timestamps -> {csv_path}")

    # Save WAV of the slice
    if WAV_OUT:
        wav_path = Path(WAV_OUT)
        wav_path.parent.mkdir(parents=True, exist_ok=True)
        sf.write(str(wav_path), res["y"], res["sr"])
        print(f"Saved slice WAV -> {wav_path}")

if __name__ == "__main__":
    main()
