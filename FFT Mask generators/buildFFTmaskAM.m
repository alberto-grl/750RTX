function Hd = buildFFTmaskAM(N, Fs, fftsize)
%function Hd = buildFFTmaskAM(N, Fs, fftsize)
%                                                Alberto I2PHD June 2013
%                                                              March 2014 
% Adapted for Octave. Alberto I4NZX June 2020
%
% scrive su file la maschera per una fast convolution fatta con la FFT.
% Il suo kernel è reale, quindi origina una maschera specchiata rispetto
% sull'asse f=0, adatta per un segnale AM
%
% N       = numero di punti del kernel, pari a fftsize - bufsize + 1 (513) 
% Fs      = sampling frequency (27901.786 Hz)
% fpass   = frequenza di taglio del passabasso, quindi la maschera
%           va da -fpass a +fpass
% fstop   = frequenza di inizio della stopband (impostata a fpass + 120)
% fftsize = size della FFT, che determina la estensione con zeri del
%           kernel


pkg load signal;
CurrentPath = fileparts(mfilename('fullpath'));

Wpass = 1;     % Passband Weight
Wstop = 40;    % Stopband Weight
dens  = 20;    % Density Factor


NumFilt = 2;
BW=[3000 5500];

N = 513; % AG Test
Fs = 31250; % ADC 128 MHZ
fftsize = 1024;

realf = zeros(NumFilt, 1024);
imagf = zeros(NumFilt, 1024);

fnameR = sprintf( '%s\\FFTmaskAM_R.txt', CurrentPath);
fnameI = sprintf( '%s\\FFTmaskAM_I.txt', CurrentPath);
fidR=fopen(fnameR, 'w');
fidI=fopen(fnameI, 'w');

for iter=1:1:NumFilt
    fpass=BW(iter); fstop = fpass + 120;
msg = sprintf('Costruzione del filtro AM da %d Hz', fpass);
disp(msg);

% Calcola i coefficienti usando la funzione FIRPM
%b  = firpm(N-1, [0 fpass fstop Fs/2]/(Fs/2), [1 1 0 0], [Wpass Wstop], {dens});
b  = remez(N-1, [0 fpass fstop Fs/2]/(Fs/2), [1 1 0 0], [Wpass Wstop], dens);

% Hd(iter) = dfilt.dffir(b);
z=zeros( fftsize - N, 1);
% bz=[b z];
bz=[b ; z];
fvtool(bz, 10, 'Mag', 'Log');
f=fft(bz);
realf(iter,:)=real(f);
imagf(iter,:)=imag(f);
end

for iter=1:1:NumFilt
if(fidR > 0)
    if(iter == 1)
      fprintf(fidR, 'const float FFTmaskAM_R[%d][1024] =\n{\n', NumFilt);
    end

    fprintf(fidR, '// AM_R PB width %d Hz\n{\n', BW(iter));
    fprintf(fidR, '%13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, \n', realf(iter,:));
    if(iter == NumFilt) fprintf(fidR, '}\n};\n');
    else  fprintf(fidR, '},\n');
    end    
else
    disp('Errore aprendo il file FFTmaskAM_R.txt');
end    
end
fclose(fidR);

for iter=1:1:NumFilt
if(fidI > 0)
    if(iter == 1)
      fprintf(fidI, 'const float FFTmaskAM_I[%d][1024] =\n{\n', NumFilt);
    end  
    fprintf(fidI, '// AM_I PB width %d Hz\n{\n', BW(iter));
    fprintf(fidI, '%13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, %13.10ff, \n', imagf(iter,:));
    if(iter == NumFilt) fprintf(fidI, '}\n};\n');
    else  fprintf(fidI, '},\n');
    end    
else
    disp('Errore aprendo il file FFTmaskAM_I.txt');
end    
end
fclose(fidI);
end

% [EOF]
