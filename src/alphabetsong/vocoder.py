#from audiolazy import Stream
#from audiolazy import lazy_lpc as ll
#from audiolazy import lazy_filters as lf
import math
import numpy as np
import scipy.signal as sig
import scipy.io.wavfile as wav
from scikits.talkbox import lpc

import pylab as plt
    
def LoadExcitation(file_name, number_of_samples, sample_freq):

    freqs = {'A':110.0, 'B':123.42, 'C':130.81, 'D':146.83, 'E':164.81, 'F':174.61, 'G':196.00, 'a':220.0, 'b':246.94, 'c':261.63, 'd':293.64, 'e':329.63}
    freqs = {'A':220.0, 'B':246.94, 'C':261.63, 'D':293.64, 'E':329.63, 'F':349.23, 'G':392.00, 'a':220.0, 'b':246.94, 'c':261.63, 'd':293.64, 'e':329.63}

    f = open(file_name, "r")
    lines = f.readlines()
    
    ex_base = np.zeros(sample_freq/60) #60 is the lowest pitch we'll use
    ex_base[0] = 1
    
    excitation = np.zeros(number_of_samples)
    
    seg_start = 0
    
    for line in lines:
        # values should be separated by the tab character
        values = line.split('\t')
        
        duration = int((float(values[1]) - float(values[0]))*sample_freq)
        note = values[2][0]
        note = note.upper()
        print note
        
        if note == 'S': # silence, don't excite
            print 'nothing to see here'
        elif note == 'U': #unvoiced excitation
            #note2 = values[2][1]
            excitation[seg_start : seg_start+duration] = 0.05*(np.random.rand(duration)-0.5)
            
            #print np.floor(duration*freqs[note2]/sample_freq)
            #template = np.tile(ex_base[0:np.floor(sample_freq/freqs[note2])], np.floor(duration*freqs[note2]/sample_freq))
            #excitation[seg_start:seg_start+len(template)] = excitation[seg_start:seg_start+len(template)] + template 
        elif note == "\n":
            print '\n'
        else:
            print np.floor(duration*freqs[note]/sample_freq)
            template = np.tile(ex_base[0:int(np.floor(sample_freq/freqs[note]))],
                               int(np.floor(duration*freqs[note]/sample_freq)))
            
            #print len(excitation)
            #print seg_start
            #print len(template)
            excitation[seg_start:seg_start+len(template)] = template 
            #excitation[seg_start+duration-1-sample_freq/freqs[note]:seg_start+duration-1] = np.zeros(sample_freq/freqs[note]);
            
        seg_start = seg_start+duration
            
    return excitation

# pull in recording from a wav file
x = wav.read("wavfiles/alphabetsong.wav")

# grab the sample freq and sound data
sample_freq = x[0]/6;
signal = sig.decimate(np.array(x[1]), 6);

#signal = signal[0:20000] # clip to save time for debugs

# setup step and window size variables
window_size = int(30*sample_freq/1000);
step_size = int(5*sample_freq/1000);

print "Sampling Frequency: %i" % sample_freq
print "Sample Count: %i" % len(signal)

#plt.plot(signal);
#plt.show();

# setup pre-emphasis and de-emphasis filters
#pre_emph = lf.ZFilter([1, -0.95], [1]);
#de_emph = 1/pre_emph;

# pre-emph filter before we do anything else
#pre_signal = np.array(list(pre_emph(signal)));
pre_signal = sig.lfilter([1, -0.95], [1], signal);


#plt.plot(pre_signal);
#plt.show();

window = np.hamming(window_size-1);
window_start = 0;
window_stop = window_size-1;

alphas = [];
gain = [];

#voicing = np.zeros(math.ceil(len(pre_signal)));
voicing = np.zeros(pre_signal.shape);


# get LPCs from each window
while(window_start+window_size<len(pre_signal)):
    
    chunk = pre_signal[window_start:window_stop]*window;
    
    #filt = ll.lpc.autocor(chunk, 16);
    filt, err, reflect_coeffs = lpc(chunk, 16);
    #print filt
    #alphas.append(filt.numerator);
    alphas.append(filt)
    
    gain.append(np.sqrt(np.sum(chunk**2)));
    
    window_start = window_start+step_size;
    window_stop = window_stop+step_size;
    
################################
# resynthesize based on the LPCs and a predefined excitation waveform
#print alphas


# prepare excitation based on annotated file with musical notes
pitch = 100 #Hz

excitation = [0 for i in range(sample_freq/pitch)];
excitation[0] = 1;

excitation = excitation*(int(math.ceil(len(pre_signal)*pitch/sample_freq)))

print "Excitation length: %i" % len(excitation);

excitation = LoadExcitation("wavfiles/alphanotes.txt", len(pre_signal), sample_freq);
print "Excitation length: %i" % len(excitation);

synth = np.zeros(len(excitation));#[0 for i in range(len(excitation))];

window_start = 0;
window_stop = window_size-1;
index = 0

while(window_start+window_size<len(excitation) and gain):
    
    chunk = excitation[window_start:window_stop];
    
    #syn_fil = lf.ZFilter([gain.pop(0)], alphas.pop(0));
    #synth_chunk = np.array(list(syn_fil(chunk)));
    synth_chunk = sig.lfilter([gain[index]], alphas[index], chunk);
    
    
    
    synth[window_start:window_stop] = synth[window_start:window_stop]+synth_chunk*window;
    
    window_start = window_start+step_size;
    window_stop = window_stop+step_size;
    index += 1;
    
plt.plot(synth);
#plt.plot(voicing*np.max(synth));
plt.plot(excitation*np.max(synth));
plt.show();

#synth = list(de_emph(synth))
synth = sig.lfilter([1], [1, -0.95], synth);

peak = max(synth)

wav.write("synth_out.wav", sample_freq, synth);
    
fo = open("wavfiles/alphabetsong.txt", "w");

for index in range(len(synth)):
    fo.write(str(10000*synth[index]/peak)+"\n")

fo.close()


