
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

#define FFTLEN 4096                     /// Number of samples to perform FFT on. Must be power of 2.

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

#define BUTTON          2

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
Audio recording/playback happens "in the background", i.e. on a different thread:
    - Whenever new audio data enters the sound card (from the microphone), the
      sound card will call a callback function and pass this data to it.
    - Whenever the sound card requires new data to send (to the speakers), it will
      call a callback function and expect to be provided with as much data as it
      requires.
These things happen at unpredictable times since they are tied to audio sample rates
etc. rather than the clock speed.
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
//MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
// Arbitrary pins
MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);
// Print function from MDMAX72xx.h library examples
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
      case 1: // binary output
      {
        current = (byte)(data[NUM_COLUMNS-col]);
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

float index2freq(int index)
{
    return 2*(float)index*(float)RATE/(float)FFTLEN;
}

float freq2index(float freq)
{
    return 0.5*freq*(float)FFTLEN/(float)RATE;
}

/**
----float mapLin2Log()----
Maps linear axis to log axis.
If graph A has a linearly scaled x-axis and graph B has a logarithmically scaled x-axis
then this function gives the x-coordinate in B corresponding to some x-coordinate in A.
**/
float mapLin2Log(float LinMin, float LinRange, float LogMin, float LogRange, float LinVal)
{
    return LogMin+(log(LinVal+1-LinMin)/log(LinRange+LinMin))*LogRange;
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
        newinputs[1] = approx_hcf(inputs+1, num_inputs-1, max_iter-1, accuracy_threshold);
        float ans = approx_hcf(newinputs, 2, max_iter-1, accuracy_threshold);
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
----Find_n_Largest()----
Finds indices of n_out largest peaks in input array.

Window should be chosen comparable to peak width, threshold like peak/noise.
**/
void Find_n_Largest(int* output, float* input, int n_out, int n_in, int window = 5, float threshold = 2)
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

/**
----------------------------
------Chord Dictionary------
----------------------------
**/
/// Checks if a chord contains ALL the input notes
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

/// Transpose a chord up by a certain number of semitones (-ve semitones_up means transpose down)
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
static bool chord_dictionary_initialized = false;

/// Initialization function: Performs transpositions to populate all_chords from A_root_chords
/// Create chord dictionary, i.e., populate all_chords[] with transposed up versions of A_root_chords
void initialize_chord_dictionary()
{
    for(int i=0; i<NUM_CHORD_TYPES; i++)
        for(int j=0; j<12; j++)
            all_chords[i*12+j] = transpose_chord(A_root_chords[i], j);
    chord_dictionary_initialized = true;
}

/// Looks for an acceptably small chord that contains ALL provided input notes, and writes the name of the chord to name_out
/// Given a set of notes (pitch numbers 1 = A, 2 = A#, 3 = B, etc.),
/// this function finds a chord that contains all those notes
/// and writes the chord's name to char* name_out
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
---------------------------
----Visualizer Function----
---------------------------
**/
void SpectralTuner(float* spectrum, int consoleWidth, int consoleHeight, bool adaptive = true,
                   float graphScale = 1.0)
{
    int numbars = consoleWidth;

    static int bargraph[NUM_COLUMNS];
    static uint8_t graph8bit[NUM_COLUMNS];
    static float octave1index[NUM_COLUMNS];                                      /// Will hold "fractional indices" in spectrum[] that map to each bar

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
    for(int i=0; i<numbars-1; i++)
    {
        float index = octave1index[i];                                          /// "Fractional index" in spectrum[] corresponding to ith frequency.
        float nextindex = octave1index[i+1];                                    /// "Fractional index" corresponding to (i+1)th frequency.
        /// OCTAVE WRAPPING
        /// To the frequency coefficient for any frequency F will be added:
        /// The frequency coefficients of all frequencies F*2^n for n=1..8
        /// (i.e., 8 octaves of the same-letter pitch)
        for(int j=0; j<4; j++)                                                  /// Iterating through 8 octaves
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

    showgraph(graph8bit,0);
}

/**
------------------
----Auto Tuner----
------------------
startAutoTuner() is a more traditional guitar tuner. It performs internal pitch
detection and displays a stationary "needle" and moving note-name "dial". Tuning
can be performed by aligning the note name to the needle.

Pitch detection is performed by finding peaks in the fft and assuming that they
are harmonics of an underlying fundamental. The approximate HCF of the frequencies
therefore gives the pitch.

span_semitones sets the span (and precision) of the dial display, i.e. how many
pitch names are to be shown on screen at once.
**/

void AutoTuner(float* spectrum, int consoleWidth, int span_semitones)
{

    int window_width = consoleWidth;

    int num_spikes = 5;                                             /// Number of fft spikes to consider for pitch deduction
    static int SpikeLocs[6];                                        /// Array to store indices in spectrum[] of fft spikes
    static float SpikeFreqs[6];                                     /// Array to store frequencies corresponding to spikes
    static char notenames[NUM_COLUMNS];

    Find_n_Largest(SpikeLocs, spectrum, num_spikes, FFTLEN/2, 40, 0.1);        /// Find spikes

    for(int i=0; i<num_spikes; i++)                                 /// Find spike frequencies (assumed to be harmonics)
        SpikeFreqs[i] = index2freq(SpikeLocs[i]);

    float pitch = approx_hcf(SpikeFreqs, num_spikes, 5, 4);         /// Find pitch as approximate HCF of spike frequencies

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

void ChordGuesser(float* spectrum, int max_notes)
{
    if(!chord_dictionary_initialized)
        initialize_chord_dictionary();

    const float quartertone = pow(2.0, 1.0/24.0);                       /// Interval of quarter-tone (used to check pitch distinctness)

    const int num_spikes = 10;                                          /// Number of fft spikes to consider
    static int SpikeLocs[11];                                          /// Array to store indices in spectrum[] of fft spikes
    static float SpikeFreqs[11];                                       /// Array to store frequencies corresponding to spikes

    float noteFreqs[10];                                                /// Array to store distinct peak frequencies
    int notes_found;                                                    /// Number of distinct peaks found

    notes_found = 0;                                                    /// Number of distinct pitches (spikes) found

    Find_n_Largest(SpikeLocs, spectrum,                                 /// Find spikes. Somehow works worse with clump rejection,
                   num_spikes, FFTLEN/2, 40, 0.1);                         /// so using separate pitch distinctness check.

    for(int i=0; i<num_spikes; i++)                                     /// Find spike frequencies
        SpikeFreqs[i] = index2freq(SpikeLocs[i]);

    noteFreqs[notes_found++] = SpikeFreqs[0];

    /// Find distinct spike frequencies and store in noteFreqs[].
    /// SpikeFreqs[] is in decreasing order of spike intensity, so the tallest spikes will be added first.
    for(int i=1; i<num_spikes; i++)                                     /// For each frequency spike
    {
        /// First check if spike is distinct
        bool distinct = true;                                           /// Assume distinct by default
        for(int j=0; j<notes_found; j++)                                /// Look at each distinct note already found,
        {
            float separation = (SpikeFreqs[i]>noteFreqs[j] ?            /// calculate the separation ratio (interval),
                                SpikeFreqs[i]/noteFreqs[j] : noteFreqs[j]/SpikeFreqs[i]);
            if(separation<quartertone)                                  /// and check that it is greater at least than a quarter tone
            {
                distinct = false;                                       /// If separation less than a quarter tone, spike is non-distinct
                break;
            }
        }

        /// Stop adding to noteFreqs if max_notes notes already found
        if(notes_found>=max_notes)
            break;

        /// If note is distinct, add it to noteFreqs.
        if(distinct)
            noteFreqs[notes_found++] = SpikeFreqs[i];
    }

    /// Sort notes found in increasing order of frequency, so "chord root" appears first.
    for(int i=0; i<notes_found; i++)
        for(int j=0; j<notes_found-1; j++)
            if(noteFreqs[j] > noteFreqs[j+1])
            {
                float tmp = noteFreqs[j];
                noteFreqs[j] = noteFreqs[j+1];
                noteFreqs[j+1] = tmp;
            }

    /// Now calculating pitch numbers (1 = A, 2 = A#, 3 = B etc.) of notes in 'chord'
    int chord_tones[10];
    int unique_chord_tones[10];
    float centsSharp;
    for(int i=0; i<notes_found; i++)
        chord_tones[i] = pitchNumber(noteFreqs[i], &centsSharp);

    /// And finding list of unique chord tones (deleting octave-up/down repetitions of notes)
    int num_unique_chord_tones = 0;
    for(int i=0; i<notes_found; i++)
    {
        bool uniq = true;
        for(int j=0; j<num_unique_chord_tones; j++)
            if(chord_tones[i] == unique_chord_tones[j])
                uniq = false;

        if(uniq)
            unique_chord_tones[num_unique_chord_tones++] = chord_tones[i];
    }

    /// Now preparing display string
    char displaystring[100];
    int chnum = 0;
    /// Add chord name
    chnum += what_chord_is(displaystring, unique_chord_tones, num_unique_chord_tones);
    /// Pad with spaces
    while(chnum<CHORD_NAME_SIZE+1) displaystring[chnum++] = ' ';
    /// Add note names
    displaystring[chnum++] = '(';
    for(int i=0; i<notes_found; i++)
    {
        chnum += pitchName(displaystring+chnum, chord_tones[i]);
        displaystring[chnum++] = ' ';
    }
    displaystring[chnum++] = ')';
    /// Null-terminate
    displaystring[chnum++] = '\0';

    /// Ad-hoc measure of peakiness of spectrum: peakiness = max/mean
    double fft_max = spectrum[0];
    double fft_mean = (double)spectrum[0]/(double)FFTLEN;
    double fft_std_dev = 0;
    for(int i=1; i<FFTLEN; i++)
    {
        fft_mean += (double)spectrum[i]/(double)FFTLEN;
        if(spectrum[i]>fft_max)
            fft_max = spectrum[i];
    }
    for(int i=1; i<FFTLEN; i++)
    {
        double diff = (spectrum[i] - fft_mean);
        fft_std_dev += diff*diff/(double)FFTLEN;
    }
    fft_std_dev = sqrt(fft_std_dev);
    double peakiness = fft_std_dev/fft_mean;

    /// Display pitches, only if spectrum was peaky (if peaky, chord has probably been played)
    if(peakiness>3)
    {
        printText(0, MAX_DEVICES-1, displaystring);
        delay(300);
    }
}

int Oscilloscope(sample* audiodata, float* spectrum, int consoleWidth, int consoleHeight)
{
  static uint8_t displaydata[NUM_COLUMNS];

  int num_spikes = 2;                                             /// Number of fft spikes to consider for pitch deduction
  static int SpikeLocs[6];                                        /// Array to store indices in spectrum[] of fft spikes
  static float SpikeFreqs[6];                                     /// Array to store frequencies corresponding to spikes
  static char notenames[NUM_COLUMNS];

  Find_n_Largest(SpikeLocs, spectrum, num_spikes, FFTLEN/2, 40, 0.1);        /// Find spikes

  for(int i=0; i<num_spikes; i++)                                 /// Find spike frequencies (assumed to be harmonics)
      SpikeFreqs[i] = index2freq(SpikeLocs[i]);

  float pitch = approx_hcf(SpikeFreqs, num_spikes, 5, 5);         /// Find pitch as approximate HCF of spike frequencies

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


  showgraph(displaydata, 3);

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

void setup()
{
  delay(1000);

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
    mode = (mode+1)%5;

    /// Flash some lights
    for (int i=1; i<=NUM_COLUMNS; i++)
    {
      mx.setColumn(i, 0b00000000);
      delay(3);
    }
    for (int i=1; i<=NUM_COLUMNS; i++)
    {
      mx.setColumn(i, 0b11111111);
      delay(3);
    }
    for (int i=1; i<=NUM_COLUMNS; i++)
    {
      mx.setColumn(i, 0b00000000);
      delay(3);
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
    case 0: SpectralTuner(spectrum, 32, 8); break;
    case 1: ChordGuesser(spectrum, 3); break;       /// Guess 3-note chords
    case 2: Oscilloscope(workingaudio, spectrum, 32,8); break;
    case 3: AutoTuner(spectrum, 9, 1); break;
    case 4: ChordGuesser(spectrum, 5); break;       /// Guess 5-note i.e. all chords
  }

}

