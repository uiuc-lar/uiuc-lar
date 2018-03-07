import scipy.io.wavfile as wav
import glob, os

os.chdir("lyrics")
for _file in glob.glob('*.wav'):
    print _file
            
    # convert wav file to a document full of strings that can be read into the icub stuff more easily
    x = wav.read(_file)

    fo = open("../synth/"+_file[:-3]+"txt", "w");

    print x
    for index in range(len(x[1])):
        fo.write(str(x[1][index][0])+"\n")

    fo.close()
