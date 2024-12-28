/*------------------------------------------------------------------
--------Raspberri Pi Pico Music Visualizer, Harmony Analyzer--------
--------------------------------------------------------------------

    Analog mic -> Raspberri Pi Pico -> MAX7219 -> 8*32 LED Matrix
                          |
                  Mode toggle button

*/

#include <MD_MAX72xx.h>
#include <SPI.h>
#include <hardware/adc.h>
#include <math.h>
#include <complex>
#include <arduinoFFT.h>

// AUDIO PROPERTIES
#define RATE              10000         /// Sample rate
#define CHUNK             64            /// Buffer size
#define RATE_MuS          100           /// = 10^6 / RATE
#define MAX_SAMPLE_VALUE  32767         /// Max sample value based on sample datatype

#define ADC_PIN           26
#define ADC_CHANNEL       0

#define FFTLEN 2048                     /// Number of samples to perform FFT on. Must be power of 2.

typedef uint16_t sample;                /// Datatype of samples.

// FFT object
/* Create FFT object */
float spectrum[FFTLEN];
float phase[FFTLEN];
ArduinoFFT<float> FFT = ArduinoFFT<float>(spectrum, phase, FFTLEN, RATE);

// DISPLAY PROPERTIES
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW             /// varies by display manufacturer
#define MAX_DEVICES 4
#define NUM_COLUMNS 32
// Must use default rpi pico pin numbers
#define CLK_PIN   18  // or SCK
#define DATA_PIN  19  // or MOSI
#define CS_PIN    17  // or SS

// Text parameters
#define CHAR_SPACING  1 // pixels between characters

// Musical literals
#define CHORD_NAME_SIZE 15
#define CHORD_MAX_NOTES 5
#define NUM_CHORD_TYPES 11      // needs to be 11?? or odd?? (probable bug)

// UI info
#define BUTTON          2       // Button on pin 2
#define NUM_MODES       7

/**
Notes/pitches are represented by pitch numbers:
1 = A, 2 = A#, 3 = B etc.
All chords must be defined in root position.
**/

struct chord
{
    int num_notes;
    int notes[CHORD_MAX_NOTES];                                             /// Must be in root position
    char name[CHORD_NAME_SIZE];

    bool contains(int notes_in[], int num_notes_in);
};

/**
------------------------
----class AudioQueue----
------------------------
Why audio queues are needed:
Audio recording/playback happens "in the background", i.e. on a different thread.
Intermittently, the sound card will transfer the data out of its working buffer to
memory shared by the main thread.
This happens at unpredictable times since it are tied to audio sample rates etc.
rather than the clock speed.
Reading and writing audio data from and to a queue helps prevent threading problems
such as skipping/repeating samples or getting more or less data than expected.
**/

class AudioQueue
{
    int len;                                                            /// Maximum length of queue
    sample *audio;                                                      /// Pointer to audio data array
    int inpos;                                                          /// Index in audio[] of back of queue
    int outpos;                                                         /// Index in audio[] of front of queue
  public:
    AudioQueue(int QueueLength = 10000);                                /// Constructor. Takes maximum length.
    ~AudioQueue();
    bool data_available(int n_samples = 1);                             /// Check if the queue has n_samples of data in it.
    bool space_available(int n_samples = 1);                            /// Check if the queue has space for n_samples of new data.
    void push(sample* input, int n_samples, float volume=1);            /// Push n_samples of new data to the queue
    void pop(sample* output, int n_samples, float volume=1);            /// Pop n_samples of data from the queue
    void peek(sample* output, int n_samples, float volume=1);           /// Peek the n_samples that would be popped
    void peekFreshData(sample* output, int n_samples, float volume=1);  /// Peek the freshest n_samples (for instantly reactive FFT)
    void peekFreshData(float* output, int n_samples, float volume=1);   /// Peek the freshest n_samples and write to float array
};

AudioQueue::AudioQueue(int QueueLength)                                     /// Constructor. Takes maximum length.
{
    len = QueueLength;
    audio = new sample[len];                                                /// Initializing audio data array.
    inpos = 0;                                                              /// Front and back both set to zero. Setting them 1 sample apart doesn't really make sense
    outpos = 0;                                                             /// because several samples will be pushed or popped at once.
}
AudioQueue::~AudioQueue()
{
    delete[] audio;
}
bool AudioQueue::data_available(int n_samples)                              /// Check if the queue has n_samples of data in it.
{
    if(inpos>=outpos)
        return (inpos-outpos)>=n_samples;
    else
        return (inpos+len-outpos)>=n_samples;
}
bool AudioQueue::space_available(int n_samples)                             /// Check if the queue has space for n_samples of new data.
{
    if(inpos>=outpos)
        return (outpos+len-inpos)>n_samples;
    else
        return (outpos-inpos)>n_samples;
}
void AudioQueue::push(sample* input, int n_samples, float volume)           /// Push n_samples of new data to the queue
{
    if(!space_available(n_samples))
    {
        /// Getting data into the queue is a priority
        outpos=(outpos+n_samples)%len;                                      /// If no space, make space
    }
    for(int i=0; i<n_samples; i++)
        audio[(inpos+i)%len] = input[i]*volume;
    inpos=(inpos+n_samples)%len;
}
void AudioQueue::pop(sample* output, int n_samples, float volume)           /// Pop n_samples of data from the queue
{
    if(!data_available(n_samples))
    {
        return;
    }
    for(int i=0; i<n_samples; i++)
        output[i] = audio[(outpos+i)%len]*volume;
    outpos=(outpos+n_samples)%len;
}
void AudioQueue::peek(sample* output, int n_samples, float volume)          /// Peek the n_samples that would be popped
{
    if(!data_available(n_samples))
    {
        return;
    }
    for(int i=0; i<n_samples; i++)
        output[i] = audio[(outpos+i)%len]*volume;
}
void AudioQueue::peekFreshData(sample* output, int n_samples,               /// Peek the freshest n_samples (for instantly reactive FFT)
                               float volume)
{
    if(!data_available(n_samples))
    {
        return;
    }
    for(int i=0; i<n_samples; i++)
        output[n_samples-i-1] = audio[(len+inpos-i)%len]*volume;
}
void AudioQueue::peekFreshData(float* output, int n_samples,               /// Peek the freshest n_samples into float array (for instantly reactive FFT)
                               float volume)
{
    if(!data_available(n_samples))
    {
        return;
    }
    for(int i=0; i<n_samples; i++)
        output[n_samples-i-1] = audio[(len+inpos-i)%len]*volume;
}

// SPI hardware interface
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

/*
----printText()----
Prints text, function from MDMAX72xx.h library examples
*/
void printText(uint8_t modStart, uint8_t modEnd, char *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
{
  uint8_t   state = 0;
  uint8_t   curLen;
  uint16_t  showLen;
  uint8_t   cBuf[8];
  int16_t   col = ((modEnd + 1) * COL_SIZE) - 1;

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  do     // finite state machine to print the characters in the space available
  {
    switch(state)
    {
      case 0: // Load the next character from the font table
        // if we reached end of message, reset the message pointer
        if (*pMsg == '\0')
        {
          showLen = col - (modEnd * COL_SIZE);  // padding characters
          state = 2;
          break;
        }

        // retrieve the next character form the font file
        showLen = mx.getChar(*pMsg++, sizeof(cBuf)/sizeof(cBuf[0]), cBuf);
        curLen = 0;
        state++;
        // !! deliberately fall through to next state to start displaying

      case 1: // display the next part of the character
        mx.setColumn(col--, cBuf[curLen++]);

        // done with font character, now display the space between chars
        if (curLen == showLen)
        {
          showLen = CHAR_SPACING;
          state = 2;
        }
        break;

      case 2: // initialize state for displaying empty columns
        curLen = 0;
        state++;
        // fall through

      case 3:	// display inter-character spacing or end of message padding (blank columns)
        mx.setColumn(col--, 0);
        curLen++;
        if (curLen == showLen)
          state = 0;
        break;

      default:
        col = -1;   // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));

  mx.control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

/*
----showgraph()----
Plots as a graph a NUM_COLUMNS long array of integers each valued between 1 and 8.
mode = 0 for a histogram
mode = 1 for 3-bit binary output
line graph otherwise
*/
void showgraph(uint8_t data[NUM_COLUMNS], int mode=0)
{
  mx.clear();

  for (uint8_t col=1; col<=NUM_COLUMNS; col++)
  {
    byte current;

    switch (mode)
    {
      case 0: // histogram
      {    
        switch (data[NUM_COLUMNS-col])
        {
          case 0: {current = 0B00000000; break;}
          case 1: {current = 0B10000000; break;}
          case 2: {current = 0B11000000; break;}
          case 3: {current = 0B11100000; break;}
          case 4: {current = 0B11110000; break;}
          case 5: {current = 0B11111000; break;}
          case 6: {current = 0B11111100; break;}
          case 7: {current = 0B11111110; break;}
          case 8: {current = 0B11111111; break;}
          default: {current = 0B11111111;}
        };
        break;
      }
      case 1: // show high values only (faux piano roll)
      {    
        switch (data[NUM_COLUMNS-col])
        {
          case 0: {current = 0B00010000; break;}
          case 1: {current = 0B00001000; break;}
          case 2: {current = 0B00000100; break;}
          case 3: {current = 0B00000010; break;}
          case 4: {current = 0B00000001; break;}
          case 5: {current = 0B11100000; break;}
          case 6: {current = 0B11100000; break;}
          case 7: {current = 0B11100000; break;}
          case 8: {current = 0B11100000; break;}
          default: {current = 0B00000000;}
        };
        break;
      }
      case 2: // binary output
      {
        current = (byte)(data[NUM_COLUMNS-col]);
        break;
      }
      case 3: // inverted binary output
      {
        current = (byte)(256-data[NUM_COLUMNS-col]);
        break;
      }
      default:  // line graph
      {
        current = 1<<(7-data[NUM_COLUMNS-col]);
      }
    };

    mx.setColumn(col, current);
    mx.setColumn(col-1, current);
  }
}

/*
Conversion between spectral index and frequency
*/
float index2freq(int index)
{
    return 2*(float)index*(float)RATE/(float)FFTLEN;
}
float freq2index(float freq)
{
    return 0.5*freq*(float)FFTLEN/(float)RATE;
}

/**
----float approx_hcf()----
Finds approximate HCF of numbers,
e.g. finds a fundamental frequency given an arbitrary subset of its harmonics. Must be
approximate because input will be real measured data, so not exact. Returns 0 if no HCF
is found (input is float, so 1 is not always a factor).
**/
float approx_hcf(float inputs[], int num_inputs, int max_iter, int accuracy_threshold)
{
    /// HCF of one number is itself
    if(num_inputs<=1)
        return inputs[0];

    /// Recursive call: if more than 2 inputs, return hcf(first input, hcf(other inputs))
    if(num_inputs>2)
    {
        float newinputs[2];
        newinputs[0] = inputs[0];
        newinputs[1] = approx_hcf(inputs+1, num_inputs-1, max_iter, accuracy_threshold);
        float ans = approx_hcf(newinputs, 2, max_iter, accuracy_threshold);
        return ans;
    }

    /// BASE CASE: num_inputs = 2

    /// Now using continued fractions to find a simple-integer approximation to the ratio inputs[0]/inputs[1]
    /// First, make sure the first input is bigger than the second, so that numerator > denominator
    if(inputs[0] < inputs[1])
    {
        float tmp = inputs[0];
        inputs[0] = inputs[1];
        inputs[1] = tmp;
    }
    /// Now setting up for continued fractions (iteration zero)
    float Ratio = inputs[0]/inputs[1];                                  /// Actual ratio. Greater than 1.
    int* IntegerParts = new int[max_iter];                              /// Array for continued fraction integer parts
    float fracpart = Ratio;                                             /// Fractional part (equals Ratio for iteration zero)
    bool accuracy_threshold_reached = false;                            /// Termination flag
    int int_sum = 0;                                                    /// Sum of increasing multiples of integer parts used as ad-hoc measure of accuracy
    int n;                                                              /// Termination point of continued fraction (incremented in loop).
    /// Calculating integer parts...
    for(n=0; n<max_iter; n++)                                           /// Limit on iterations ensures simple integer ratio (or nothing)
    {
        IntegerParts[n] = (int)fracpart;
        int_sum += n*IntegerParts[n];
        fracpart = 1.0/(fracpart - IntegerParts[n]);

        if(int_sum > accuracy_threshold)
        {
            accuracy_threshold_reached = true;
            break;
        }
    }

    /// Now finding the simple integer ratio that corresponds to the integer parts found
    int numerator = 1;
    int denominator = IntegerParts[n-1];
    for(int i=n-1; i>0; i--)
    {
        int tmp = denominator;
        denominator = denominator*IntegerParts[i-1] + numerator;
        numerator = tmp;
    }

    delete[] IntegerParts;

    /// If simple integer ratio not found return 0
    if(!accuracy_threshold_reached)
        return 0;

    /// If simple integer ratio successfully found, then the HCF is:
    /// the bigger input divided by the numerator (which is bigger than the denominator)
    /// or
    /// the smaller input divided by the denominator (which is smaller than the numerator)
    /// Now returning the geometric mean of these two possibilities.
    return sqrt((inputs[0]/(float)numerator)*((inputs[1]/(float)denominator)));
}

/**
----Find_n_Peaks()----
Finds indices of n_out largest peaks in input array.
Results in increasing order of frequency.
Window should be chosen comparable to peak width, threshold like peak/noise.
**/
void Find_n_Peaks(int* output, float* input, int n_out, int n_in, int window = 5, float threshold = 2)
{
    /// Array to store averaged spike heights
    float* outputvals = new float[n_out];

    /// First, find the position of the smallest element in the input array
    /// and the average squared difference between consecutive inputs
    int min_pos = 0;
    float scatter = 0;
    for(int i=1; i<n_in; i++)
    {
        scatter += (input[i]-input[i-1])*(input[i]-input[i-1])/n_in;
        if(input[i]<input[min_pos])
            min_pos = i;
    }
    scatter = sqrt(scatter);

    /// And set every element of the output to this minimum element
    for(int i=0; i<n_out; i++)
    {
        output[i] = min_pos;
        outputvals[i] = input[min_pos];
    }

    /// Peak detection algorithm inspired by S Huber's
    /// Initialize mov_avg to average of first window elements of input
    float mov_avg = 0;
    for (int i=0; i<window; i++)
      mov_avg += input[i]/window;

    /// Now iterate through input, updating mov_avg and checking for rising edges of peaks
    for (int i=window; i<n_in; i++)
    {
      /// If rising edge of peak detected, skip forward until not on rising edge
      if (input[i]-mov_avg > threshold*scatter)
      {
        int peak_start, peak_end, peak_loc;
        float peak_val = 0;
        /// Store peak location
        peak_start = i;
        /// Skip forward off of rising edge
        while (input[i]-mov_avg > threshold*scatter)
        {
          /// updating total input value over rising-edge region
          peak_val += input[i];

          i++;  // Skip forward
          /// Updating moving average while skipping forward
          mov_avg -= input[i-window]/window;
          mov_avg += input[i]/window;
        }
        peak_end = i;
        peak_loc = (peak_start + peak_end)/2;
        peak_val /= (peak_end-peak_start);    // Average

        /// Insert peak into output
        int i1o;    /// Output index

        /// If the current peak is greater than any output
        for (i1o=0; i1o<n_out; i1o++)
          if (peak_val>outputvals[i1o+1])
          {
            if (i1o == 0 || peak_val<outputvals[i1o])
            {
                /// shift that output and everything to its right, to the right
                for (int j=i1o; j<n_out-1; j++)
                {
                  output[j+1] = output[j];
                  outputvals[j+1] = outputvals[j];
                }
                /// and insert the current peak at the old location of "that output".

                output[i1o] = peak_loc;
                outputvals[i1o] = peak_val;
            }

          }

      }
      /// Updating moving average
      mov_avg -= input[i-window]/window;
      mov_avg += input[i]/window;
    }

    delete[] outputvals;
}

/*
----detect_pitch()----
Detects pitch considering num_spikes spectral peaks
*/
float detect_pitch(float* spectrum, int num_spikes)
{
  num_spikes = num_spikes < 10 ? num_spikes : 10;
  static int SpikeLocs[11];                                      /// Array to store indices in spectrum[] of fft spikes
  static float SpikeFreqs[11];                                   /// Array to store frequencies corresponding to spikes

  Find_n_Peaks(SpikeLocs, spectrum, num_spikes, FFTLEN/2, 40, 0.1);        /// Find spikes

  for(int i=0; i<num_spikes; i++)                                 /// Find spike frequencies (assumed to be harmonics)
      SpikeFreqs[i] = index2freq(SpikeLocs[i]);

  float pitch = approx_hcf(SpikeFreqs, num_spikes, 5, 5);         /// Find pitch as approximate HCF of spike frequencies
  return pitch;
}

/**
----int pitchNumber()----
Given a frequency freq, this function finds and returns the closest pitch number,
which is an integer between 1 and 12 where 1 refers to A and 12 to G#.
The number of cents between the input frequency and the 'correct' pitch is written
to centsSharp
**/
int pitchNumber(float freq, float *centsSharp)
{
    const double semitone = pow(2.0, 1.0/12.0);
    const double cent = pow(2.0, 1.0/1200.0);

    /// First, octave shift input frequency to within 440Hz and 880Hz (A4 and A5)
    while(freq<440)
        freq*=2;
    while(freq>880)
        freq/=2;

    /// "Log base semitone" of ratio of frequency to A4
    /// i.e., Number of semitones between A4 and freq
    int pitch_num = round(log(freq/440.0)/log(semitone));

    /// Enforcing min and max values of pitch_num
    if(pitch_num>11)
        pitch_num = 11;
    if(pitch_num<0)
        pitch_num = 0;

    /// "Log base cent" of ratio of freq to 'correct' (i.e. A440 12TET) pitch
    if(centsSharp != nullptr)
        *centsSharp = log(freq/(440.0*pow(semitone, (double)(pitch_num))))/log(cent);

    return pitch_num+1;                                                 /// Plus 1 so that 1 corresponds to A (0 = error).
}

/**
----int pitchName()----
Given a pitch number from 1 to 12 where 1 refers to A and 12 to G#
This function writes the pitch name to char* name and returns the length of the name.
**/
int pitchName(char* name, int pitch_num)
{
    switch(pitch_num)
    {
        case 1 : name[0] = 'A'; return 1;
        case 2 : name[0] = 'A'; name[1] = '#'; return 2;
        case 3 : name[0] = 'B'; return 1;
        case 4 : name[0] = 'C'; return 1;
        case 5 : name[0] = 'C'; name[1] = '#'; return 2;
        case 6 : name[0] = 'D'; return 1;
        case 7 : name[0] = 'D'; name[1] = '#'; return 2;
        case 8 : name[0] = 'E'; return 1;
        case 9 : name[0] = 'F'; return 1;
        case 10: name[0] = 'F'; name[1] = '#'; return 2;
        case 11: name[0] = 'G'; return 1;
        case 12: name[0] = 'G'; name[1] = '#'; return 2;
        default: return 0;
    }
}

/*
----------------------------
------Chord Dictionary------
----------------------------

----chord::contains()----
Checks if a chord contains ALL the input notes
*/
bool chord::contains(int notes_in[], int num_notes_in)
{
    for(int i=0; i<num_notes_in; i++)
    {
        bool contains_ith = false;
        for(int j=0; j<num_notes; j++)
            if(notes[j] == notes_in[i])
            {
                contains_ith = true;
                break;
            }
        if(!contains_ith)
            return false;
    }
    return true;
}

/*
----transpose_chord()----
Transposes a chord up by a certain number of semitones (-ve semitones_up means transpose down)
*/
chord transpose_chord(chord old_chord, int semitones_up)
{
    /// If shift is zero return chord unchanged
    if(semitones_up == 0)
        return old_chord;
    /// If shift is more than 11 semitones down the arithmetic doesn't work
    else if(semitones_up < -11)
    {
        return old_chord;
    }

    /// Create a new chord to return
    chord new_chord;

    /// Number of notes is unchanged through transposition
    new_chord.num_notes = old_chord.num_notes;

    /// Iterate through notes and transpose each one
    for(int i=0; i<old_chord.num_notes; i++)
        new_chord.notes[i] = (11 + old_chord.notes[i] + semitones_up)%12 + 1;

    /// Copy the old chord name to the new chord,
    /// then replace the first two characters of the name with the appropriate letter name
    /// i.e., with the letter name corresponding to the first element of the chord
    for(int i=0; i<CHORD_NAME_SIZE; i++)
        new_chord.name[i] = old_chord.name[i];

    new_chord.name[1] = ' ';
    pitchName(new_chord.name, new_chord.notes[0]);

    return new_chord;
}

/**
A_root_chords will be transposed up to find all other chords.
i.e., A_root_chords defines all chord types that the program will look for.

NUM_CHORD_TYPES should reflect how many chords are in the array below.

All chords must be defined in root position.
**/
static chord A_root_chords[] = {
    
    {4, {1, 5, 8, 3}, "A +9"},
    {4, {1, 5, 8, 11}, "A Mm7"},
    {4, {1, 4, 8, 12}, "A mM7"},
    {4, {1, 5, 8, 12}, "A M7"},
    {4, {1, 4, 8, 11}, "A m7"},
    {3, {1, 4, 7}, "A dim"},
    {3, {1, 6, 8}, "A sus4"},
    {3, {1, 3, 8}, "A sus2"},
    {3, {1, 5, 8}, "A Maj"},
    {3, {1, 4, 8}, "A min"},
    {2, {1, 8}, "A "}

};

/// After initialization, this will hold all transpositions of A_root_chords
static chord all_chords[NUM_CHORD_TYPES*12];

/// Initialization flag
bool chord_dictionary_initialized = false;

/*
----initialize_chord_dictionary()----
Initialization function: Performs transpositions to populate all_chords from A_root_chords
Creates "chord dictionary" in RAM.
*/
void initialize_chord_dictionary()
{
    for(int i=0; i<NUM_CHORD_TYPES; i++)
        for(int j=0; j<12; j++)
            all_chords[i*12+j] = transpose_chord(A_root_chords[i], j);
    chord_dictionary_initialized = true;
}

/*
----what_chord_is()----
Looks for an acceptably small chord that contains ALL provided input
notes, and writes the name of the chord to name_out.
Given a set of notes (pitch numbers 1 = A, 2 = A#, 3 = B, etc.),
this function finds a chord that contains all those notes and writes
the chord's name to char* name_out
Guesses prioritized based on fewest undetected (inferred) notes and
matching the root (lowest) note
*/
int what_chord_is(char* name_out, int notes[], int num_notes)
{
    int chord_index;                                                /// This will store the guess (its index in all_chords[])

    /// Iterate through all_chords looking for chords that contains all input notes
    int candidates[NUM_CHORD_TYPES*12];
    int num_candidates = 0;
    for(int i=0; i<NUM_CHORD_TYPES*12; i++)
        /// If an acceptably small chord is found that contains all input notes, add its index to candidates[]
        if(all_chords[i].contains(notes, num_notes) && all_chords[i].num_notes<=num_notes)
            candidates[num_candidates++] = i;

    /// If not found return 0
    if(num_candidates == 0)
        return 0;

    /// If there is exactly one candidate, it is the answer.
    if(num_candidates == 1)
        chord_index = candidates[0];
    /// Otherwise, trim down the list of candidates by the following criteria (in order of importance):
    /// 1. Based on chord::num_notes (smaller preferred).
    /// 2. Based on root note.
    else
    {
        /// Find minimum chord size in list of candidates
        int min_size = all_chords[candidates[0]].num_notes;
        for(int i=0; i<num_candidates; i++)
            if(all_chords[candidates[i]].num_notes < min_size)
                min_size = all_chords[candidates[i]].num_notes;
        /// Disregard (delete) all candidates with more notes than min_size
        for(int i=0; i<num_candidates; i++)
            if(all_chords[candidates[i]].num_notes > min_size)
            {
                for(int j=i; j<num_candidates-1; j++)
                    candidates[j] = candidates[j+1];
                num_candidates--;
            }
        /// Now guess the first remaining candidate by default
        chord_index = candidates[0];
        /// But if the root note of some other candidate matches the first input note, guess that instead.
        for(int i=0; i<num_candidates; i++)
            if(all_chords[candidates[i]].notes[0] == notes[0])
                chord_index = candidates[i];

    }

    /// Write name of chord found to name_out
    int name_length = 0;
    while(all_chords[chord_index].name[name_length] != '\0')
    {
        name_out[name_length] = all_chords[chord_index].name[name_length];
        name_length++;
    }

    /// Return name length so that calling function known how many characters were written
    return name_length;
}

/**
-------------------------
----NotesVisualizer()----
-------------------------
Displays a spectral histogram wrapped horizontally at the octave with an optionally adaptive
vertical scale
Some function arguments untested, inherited from PC version of code.
**/
void NotesVisualizer(float* spectrum, int consoleWidth, int consoleHeight, int displaymode, 
                   bool adaptive = true, float graphScale = 1.0)
{
    int numbars = consoleWidth;

    static int bargraph[NUM_COLUMNS];
    static uint8_t graph8bit[NUM_COLUMNS];
    static float octave1index[NUM_COLUMNS+1];                                    /// Will hold "fractional indices" in spectrum[] that map to each bar

    /// SETTING FIRST-OCTAVE INDICES
    /// The entire x-axis of the histogram is to span one octave i.e. an interval of 2.
    /// Therefore, each bar index increment corresponds to an interval of 2^(1/numbars).
    /// The first frequency is A1 = 55Hz.
    /// So the ith frequency is 55Hz*2^(i/numbars).
    for(int i=0; i<numbars+1; i++)
        octave1index[i] = freq2index(220.0*pow(2,(float)i/numbars));             /// "Fractional index" in spectrum[] corresponding to ith frequency.

    for(int i=0; i<numbars; i++)
        bargraph[i]=0;

    /// PREPARING TUNER HISTOGRAM
    /// Iterating through log-scaled output indices and mapping them to linear input indices
    /// (instead of the other way round).
    /// So, an exponential mapping.
    for(int i=0; i<numbars; i++)
    {
        float index = octave1index[i];                                          /// "Fractional index" in spectrum[] corresponding to ith frequency.
        float nextindex = octave1index[i+1];                                    /// "Fractional index" corresponding to (i+1)th frequency.
        /// OCTAVE WRAPPING
        /// To the frequency coefficient for any frequency F will be added:
        /// The frequency coefficients of all frequencies F*2^n for n=1..4
        /// (i.e., 4 octaves of the same-letter pitch)
        for(int j=0; j<4; j++)                                                  /// Iterating through 4 octaves
        {
            /// Add everything in spectrum[] between current index and next index to current histogram bar.
            for(int k=round(index); k<round(nextindex); k++)                    /// Fractional indices must be rounded for use
                /// There are (nextindex-index) additions for a particular bar, so divide each addition by this.
                bargraph[i]+=0.02*spectrum[k]/(nextindex-index);

            /// Frequency doubles with octave increment, so index in linearly spaced data also doubles.
            index*=2;
            nextindex*=2;
        }
    }

    if(adaptive)
    {
        int maxv = bargraph[0];
        for(int i=0; i<numbars; i++)
        {
            if(bargraph[i]>maxv)
                maxv=bargraph[i];
        }
        graphScale = 1.0/(float)maxv;
        for(int i=0; i<numbars; i++)
            graph8bit[i] = (consoleHeight*bargraph[i])*graphScale;
    }

    showgraph(graph8bit,displaymode);
}

/**
------------------
----Auto Tuner----
------------------
startAutoTuner() is a more traditional guitar tuner. It performs internal pitch
detection and displays a moving note-name "dial".

Pitch detection is performed by finding peaks in the fft and assuming that they
are harmonics of an underlying fundamental. The approximate HCF of the frequencies
therefore gives the pitch.

span_semitones sets the span (and precision) of the dial display, i.e. how many
pitch names are to be shown on screen at once.
**/
void AutoTuner(float* spectrum, int consoleWidth, int span_semitones)
{

    int window_width = consoleWidth;
    char notenames[NUM_COLUMNS];

    float pitch = detect_pitch(spectrum, 5);                        /// Detect pitch from 5 largest peaks in FFT

    if(pitch)                                                       /// If pitch found, update notenames and print
    {
        for(int i=0; i<window_width; i++)                           /// First initialize notenames to all whitespace
            notenames[i] = ' ';

        /// Find pitch number (1 = A, 2 = A# etc.) and how many cents sharp or flat (centsOff<0 means flat)
        float centsOff;
        int pitch_num = pitchNumber(pitch, &centsOff);

        /// Find appropriate location for pitch letter name based on centsOff
        /// (centsOff = 0 means "In Tune", location exactly in the  middle of the window)
        int loc_pitch = window_width/2 + centsOff*span_semitones*window_width/200;

        /// Write letter name corresponding to current pitch to appropriate location
        pitchName(notenames+loc_pitch, pitch_num);

        notenames[window_width] = '\0';                             /// Terminate string

        printText(0, MAX_DEVICES-1, notenames);
    }
}

/*
----------------------
----ChordGuesser()----
----------------------
Finds total content of each pitch class in the provided spectrum and attempts
to guess a chord with max_notes notes or fewer, by calling what_chord_is().
Sentivity should be between 0 and 1. Zero means never guess anything and 1
means always try to guess something even if you're wrong.
*/
void ChordGuesser(float* spectrum, int max_notes, float sensitivity = 0.5)
{
    if(!chord_dictionary_initialized)
        initialize_chord_dictionary();

    static float notecontent[12];                                                /// notecontent[0] will store detected intensity of A, notecomtent[2] of A# etc.
    static float octave1index[13];                                               /// Will hold "fractional indices" in spectrum[] that map to each bar

    /// SETTING FIRST-OCTAVE INDICES
    /// Each index increment corresponds to an interval of 2^(1/12).
    /// The first frequency is a quarter tone below A1 i.e. below 110Hz.
    /// So the ith frequency is 110Hz*2^((2*i-1)/24)
    for(int i=0; i<13; i++)
        octave1index[i] = freq2index(110.0*pow(2,(float)(2*i-1)/24));             /// "Fractional index" in spectrum[] corresponding to ith frequency.

    for(int i=0; i<12; i++)
        notecontent[i]=0;

    /// Finding intensity of each note in input
    /// Iterating through log-scaled output indices and mapping them to linear input indices
    /// (instead of the other way round).
    /// So, an exponential mapping.
    for(int i=0; i<12; i++)
    {
        float index = octave1index[i];                                          /// "Fractional index" in spectrum[] corresponding to ith frequency.
        float nextindex = octave1index[i+1];                                    /// "Fractional index" corresponding to (i+1)th frequency.
        /// OCTAVE WRAPPING
        /// To the frequency coefficient for any frequency F will be added:
        /// The frequency coefficients of all frequencies F*2^n for n=1..4
        /// (i.e., 4 octaves of the same-letter pitch)
        for(int j=0; j<4; j++)                                                  /// Iterating through 4 octaves
        {
            /// Add everything in spectrum[] between current index and next index to current histogram bar.
            for(int k=round(index); k<round(nextindex); k++)                    /// Fractional indices must be rounded for use
                /// There are (nextindex-index) additions for a particular bar, so divide each addition by this.
                notecontent[i]+=spectrum[k]/(4*(nextindex-index));

            /// Frequency doubles with octave increment, so index in linearly spaced data also doubles.
            index*=2;
            nextindex*=2;
        }
    }


    /// chord_tones holds pitch numbers that will be sorted in decreasing order of detected intensity    
    int chord_tones[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
    /// Now populating chord_tones by sorting found notes in order of intensity
    /// Eventually we will include a chord guessing preference for loud root notes. Low = loud, often, since octave harmonics add
    for (int i=0; i<12; i++)
        for (int j=0; j<12; j++)
            if (notecontent[chord_tones[i]-1] > notecontent[chord_tones[j]-1])
            {
              int tmp = chord_tones[i];
              chord_tones[i] = chord_tones[j];
              chord_tones[j] = tmp;
            }
    
    /// The number of notes in the chord played is probably smaller than the max notes allowable.
    /// Now iterating backward (ascending intensity) through chord_tones to find the transition point from below-average intensity to above-average intensity
    /// All entries in chord_tones to the left of the sudden jump are part of the chord played.
    int notes_found = max_notes;
    float mean = 0;
    float max = 0;
    for (int i=0; i<12; i++)
    {
        mean += notecontent[chord_tones[i]-1]/12.0;
        if (notecontent[chord_tones[i]-1] > max)
          max = notecontent[chord_tones[i]-1];
    }
    float thresh =  max - sensitivity*(max-mean);
    while ( notecontent[chord_tones[notes_found]-1] < thresh )
    {
      notes_found--;
      if (notes_found<=0)
      {
        notes_found = max_notes;          /// But if no jump is found all notes are important
        break;
      }
    }

    /// Now preparing display string
    char displaystring[100];
    int chnum = 0;
    /// Show chord name if found
    int NameLength = what_chord_is(displaystring, chord_tones, notes_found);      /// if chord found write chord name to displaystring
    if (NameLength)         /// If chord found
    {
        // null terminate
        displaystring[NameLength] = '\0';
        // print and pause
        printText(0, MAX_DEVICES-1, displaystring);
    }
}

/*
----------------------
----Oscilloscope()----
----------------------
This detects pitch from the provided spectrum, then plots a certain number of cycles of the
provided audio data to the screen as a 3-bit oscilloscope with adaptive vertical scale.
*/
int Oscilloscope(sample* audiodata, float* spectrum, int consoleWidth, int consoleHeight)
{
  static uint8_t displaydata[NUM_COLUMNS];

  float pitch = detect_pitch(spectrum, 5);

  int period_nsamps = freq2index(pitch);

  int maxpos = 0;
  int minpos = 0;
  for (int i=0; i<FFTLEN; i++)
  {
    if (audiodata[i] > audiodata[maxpos])
      maxpos = i;
    if (audiodata[i] < audiodata[minpos])
      minpos = i;
  }

  for (int i = 0; i<consoleWidth; i++)
    displaydata[i] = 8 * (audiodata[i*2*period_nsamps/(consoleWidth*2)] - audiodata[minpos]) / (audiodata[maxpos] - audiodata[minpos]);


  showgraph(displaydata, 5);

  return period_nsamps;
}

/*--------------------------------------------------
-- CORE 2 (setup1, loop1) WILL ACT AS A SOUNDCARD --
--------------------------------------------------*/
AudioQueue MainAudioQueue(20000);         /// Main Audio Queue
/// ADC Setup on pico core 2
sample ADCbuffer[CHUNK];
int wptr = 0;                             /// ADC buffer write pointer
int cur_time, prev_time;
// SOUNDCARD SETUP: INITIALIZE ADC
void setup1()
{
  // Setup the ADC
  adc_init();
  // Select ADC input 0 (GPIO26)
  adc_gpio_init(ADC_PIN);
  adc_select_input(ADC_CHANNEL);
  prev_time = micros();
}
// SOUNDCARD OPERATION:
// constantly check time and read value into buffer at regular intervals
// push audio to queue if buffer full
void loop1()
{  
  // If ADC buffer is full push data to queue and clear buffer
  if (wptr>=CHUNK)
  {
    MainAudioQueue.push(ADCbuffer, CHUNK);        // This takes a while so no delay needed
    wptr = 0;                                     // Clear buffer
  }
  else
    delayMicroseconds(1);

  // If the time is right add a new sample to the buffer
  cur_time = micros();
  if (cur_time - prev_time > RATE_MuS || cur_time < prev_time)
  {
    ADCbuffer[wptr++] = adc_read();
    prev_time = cur_time;
  }
}

/// Function to flash message during mode switch
void flash_message(char msg[])
{
  /// Flash lights
  for (int i=1; i<=NUM_COLUMNS; i++)
  {
    mx.setColumn(i, 0b00000000); delay(1);
  }
  for (int i=1; i<=NUM_COLUMNS; i++)
  {
    mx.setColumn(i, 0b11111111); delay(1);
  }
  for (int i=1; i<=NUM_COLUMNS; i++)
  {
    mx.setColumn(i, 0b00000000); delay(1);
  }
  /// Print message
  printText(0, MAX_DEVICES-1, msg); delay(100);
  /// If message not empty wipe screen
  if (msg[0]!='\0')
    for (int i=1; i<=NUM_COLUMNS; i++)
    {
      mx.setColumn(i, 0b00000000); delay(10);
    }
}

/*
--------------------------
-- CORE 1 (setup, loop) --
--------------------------
Core 1 does everything other than the audio recording, such as FFT, pitch and
chord detection, and controlling the screen.
*/

void setup()
{
  Serial.begin(9600);
  delay(1000);                    /// Let sound card (core 2) fill up the audio queue a bit
  SPI1.begin();
  mx.begin();
  initialize_chord_dictionary();
  pinMode(BUTTON, INPUT_PULLUP);
}

sample workingaudio[FFTLEN];
int mode = 0;
void loop()
{
  if (!digitalRead(BUTTON))   // If button pressed
  {
    /// Toggle mode
    mode = (mode+1)%NUM_MODES;

    /// Flash appropriate message
    switch (mode)
    {
      case 0: flash_message("Pitch"); break;
      case 1: flash_message("Piano"); break;
      case 2: flash_message("Bass"); break;
      case 3: flash_message("Triad"); break;
      case 4: flash_message("Chord"); break;
      case 5: flash_message("Osc"); break;
      case 6: flash_message("Tuner"); break;
    }
  }

  /// Read audio data into workingaudio[]
  MainAudioQueue.peekFreshData(workingaudio, FFTLEN);
  /// Copy audio to spectrum and intialize phase array
  for (int i=0; i<FFTLEN; i++)
  {
    spectrum[i] = workingaudio[i];
    phase[i] = 0;
  }
  /// FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);	/* Weigh data */
  FFT.compute(FFTDirection::Forward); /* Compute FFT */
  FFT.complexToMagnitude(); /* Compute magnitudes */

  /// Do mode thing
  switch (mode)
  {
    case 0: NotesVisualizer(spectrum, 32, 8, 0); break;
    case 1: NotesVisualizer(spectrum, 32, 8, 1); break;
    case 2: ChordGuesser(spectrum, 2); break;       /// Guess 2-note i.e. power chords
    case 3: ChordGuesser(spectrum, 3); break;       /// Guess 3-note chords
    case 4: ChordGuesser(spectrum, 4); break;       /// Guess 4-note i.e. all chords
    case 5: Oscilloscope(workingaudio, spectrum, 32,8); break;
    case 6: AutoTuner(spectrum, 9, 1); break;
  }

}

