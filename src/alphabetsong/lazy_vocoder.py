from audiolazy import Stream
from audiolazy import lazy_lpc as ll
from audiolazy import lazy_filters as lf
import math
import numpy as np
import scipy.signal as sig

import scipy.io.wavfile as wav

import pylab as plt




# pull in recording from a wav file
x = wav.read("wavfiles/wwaaya.wav")

# grab the sample freq and sound data
sample_freq = x[0]/6;
signal = sig.decimate(np.array(x[1]), 6);

# setup step and window size variables
window_size = int(30*sample_freq/1000);
step_size = int(5*sample_freq/1000);

print "Sampling Frequency: %i" % sample_freq
print "Sample Count: %i" % len(signal)

#plt.plot(signal);
#plt.show();

# setup pre-emphasis and de-emphasis filters
pre_emph = lf.ZFilter([1, -0.95], [1]);
de_emph = 1/pre_emph;

# pre-emph filter before we do anything else
pre_signal = np.array(list(pre_emph(signal)));

#plt.plot(pre_signal);
#plt.show();

window = np.hanning(window_size-1);
window_start = 0;
window_stop = window_size-1;

alphas = [];
gain = [];

# get LPCs from each window
while(window_start+window_size<len(pre_signal)):
    
    chunk = pre_signal[window_start:window_stop]*window;
    
    filt = ll.lpc.autocor(chunk, 16);
    alphas.append(filt.numerator);
    
    gain.append(math.sqrt(sum([i**2 for i in chunk])));
    
    window_start = window_start+step_size;
    window_stop = window_stop+step_size;
    
################################
# resynthesize based on the LPCs and a predefined excitation waveform

pitch = 100 #Hz

excitation = [0 for i in range(sample_freq/pitch)];
excitation[0] = 1;

excitation = excitation*(int(math.ceil(len(pre_signal)*pitch/sample_freq)))

print "Excitation length: %i" % len(excitation);

synth = np.zeros(len(excitation));#[0 for i in range(len(excitation))];

window_start = 0;
window_stop = window_size-1;
while(window_start+window_size<len(excitation) and gain):
    
    chunk = excitation[window_start:window_stop];
    
    syn_fil = lf.ZFilter([gain.pop(0)], alphas.pop(0));
    
    synth_chunk = np.array(list(syn_fil(chunk)));
    
    synth[window_start:window_stop] = synth[window_start:window_stop]+synth_chunk*window;
    
    window_start = window_start+step_size;
    window_stop = window_stop+step_size;
    
plt.plot(synth);
plt.show();

synth = list(de_emph(synth))

peak = max(synth)

#wav.write("synth_out.wav", sample_freq, synth);
    
fo = open("synth_out.txt", "w");

for index in range(len(synth)):
    fo.write(str(10000*synth[index]/peak)+"\n")

fo.close()


