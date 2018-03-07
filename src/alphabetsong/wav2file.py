import scipy.io.wavfile as wav
import glob, os

os.chdir("/lyrics")
for file in glob.glob('*.wav'):
    print file
    
# convert wav file to a document full of strings that can be read into the icub stuff more easily
x = wav.read("wavfiles/wwaaya.wav")

fo = open("somefile.txt", "w");

for index in range(len(x[1])):
    fo.write(str(x[1][index])+"\n")

fo.close()
