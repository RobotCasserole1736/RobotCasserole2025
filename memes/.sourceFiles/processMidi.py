import mido
from mido import MidiFile, MidiTrack, Message, MetaMessage


# Configurable constants
N = 4  # Number of output tracks
fname = "call_me_maybe"
INPUT_FILE = f"{fname}.mid"  # Path to the input MIDI file
OUTPUT_FILE = f"{fname}_proc.mid"  # Path to the output MIDI file


def get_bpm(midi):
    """Extract the BPM from the MIDI file. Default to 120 if not specified."""
    for track in midi.tracks:
        for msg in track:
            if msg.type == 'set_tempo':
                return mido.tempo2bpm(msg.tempo)
    return 120  # Default BPM

def combine_tracks(midi):
    """Combine all tracks in a MIDI file into a single track."""
    combined_track = MidiTrack()
    all_events = []

    for track in midi.tracks:
        abs_time = 0
        for msg in track:
            abs_time += msg.time
            all_events.append((abs_time, msg))

    all_events.sort(key=lambda x: x[0])  # Sort by absolute time

    current_time = 0
    for abs_time, msg in all_events:
        delta_time = abs_time - current_time
        current_time = abs_time
        new_msg = msg.copy(time=delta_time)
        combined_track.append(new_msg)

    return combined_track

def split_into_monophonic_tracks(combined_track, num_tracks, ticks_per_beat):
    """Split a polyphonic track into monophonic tracks."""
    monophonic_tracks = [MidiTrack() for _ in range(num_tracks)]
    active_tracks = [0] * num_tracks  # Track the end time of the last note in each track

    active_notes = {}  # Track currently active notes as {note: (start_time, track_index)}

    current_time = 0
    for msg in combined_track:
        current_time += msg.time

        if msg.type == 'note_on' and msg.velocity > 0:
            # Add the note to active notes
            active_notes[msg.note] = (current_time, None)
            sorted_notes = sorted(active_notes.items(), key=lambda x: -x[0])  # Sort by note value (highest first)

            for i, (note, (start_time, assigned_track)) in enumerate(sorted_notes):
                if i == 0 and assigned_track is None:
                    # Always preserve the highest note
                    available_track = 0
                else:
                    # Find the first available track or the one with the earliest end time
                    available_track = min(range(1, num_tracks), key=lambda t: active_tracks[t])

                if active_tracks[available_track] > start_time:
                    # Shorten any overlapping note
                    overlap_msg = Message('note_off', note=monophonic_tracks[available_track][-1].note, time=0)
                    monophonic_tracks[available_track].append(overlap_msg)

                if assigned_track is None:
                    # Assign the note to the available track
                    delta_time = current_time - active_tracks[available_track]
                    monophonic_tracks[available_track].append(msg.copy(time=delta_time))
                    active_notes[note] = (start_time, available_track)
                    active_tracks[available_track] = current_time

        elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
            # Assign note-off to the track where the note started
            if msg.note in active_notes:
                _, track_index = active_notes[msg.note]
                if track_index is not None:
                    delta_time = current_time - active_tracks[track_index]
                    monophonic_tracks[track_index].append(msg.copy(time=delta_time))
                    active_tracks[track_index] = current_time
                del active_notes[msg.note]

    # Ensure all active notes are terminated
    for note, (start_time, track_index) in active_notes.items():
        if track_index is not None:
            monophonic_tracks[track_index].append(Message('note_off', note=note, time=active_tracks[track_index]))

    return monophonic_tracks

def process_midi(input_file, output_file, num_tracks):
    """Process the MIDI file to split polyphonic tracks into monophonic tracks."""
    midi = MidiFile(input_file)
    bpm = get_bpm(midi)
    ticks_per_beat = midi.ticks_per_beat

    combined_track = combine_tracks(midi)
    monophonic_tracks = split_into_monophonic_tracks(combined_track, num_tracks, ticks_per_beat)

    # Create output MIDI file
    output_midi = MidiFile()
    output_midi.ticks_per_beat = ticks_per_beat

    # Add monophonic tracks to output
    for track in monophonic_tracks:
        output_midi.tracks.append(track)

    # Add tempo track to ensure BPM is preserved
    tempo_track = MidiTrack()
    tempo_track.append(MetaMessage('set_tempo', tempo=mido.bpm2tempo(bpm)))
    output_midi.tracks.insert(0, tempo_track)

    output_midi.save(output_file)

if __name__ == "__main__":
    process_midi(INPUT_FILE, OUTPUT_FILE, N)
