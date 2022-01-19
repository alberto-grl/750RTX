/*******************************************************************************

                   FIRcoefs.h module of the program ARM_Radio
						                          
						                          Copyright 2015 by Alberto I2PHD, June 2015
						                                      
    This file is part of ARM_Radio.

    ARM_Radio is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ARM_Radio is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ARM_Radio.  It is contained in the file Copying.txt in the
    same ZIP file where this file was extracted from.				                                      
						                                      
*******************************************************************************/

/*
  Coefficients for an equiripple FIR, Fs=111.60714 kHz, Fpass=8 kHz, Fstop=13 kHz, A=80 dB
	to be used before the decimation by 4 of the output of the R16, M4 CIC
	
	                                       Alberto I2PHD, June 2015
*/

#define NUMFIRCOEFS  64

/*

TODO: needs to be optimized

Big alias signal at F out CIC / R CIC (abt 73 KHz)

FIR filter designed with
http://t-filter.appspot.com
http://t-filter.engineerjs.com/

sampling frequency: 156250 Hz
(10 MHz / 16))

* 0 Hz - 8000 Hz
  gain = 1
  desired ripple = 1 dB
  actual ripple = 2.624646859127177 dB

* 13000 Hz - 70000 Hz
  gain = 0
  desired attenuation = -80 dB
  actual attenuation = -69.20362343893176 dB





float FIRcoefs[NUMFIRCOEFS] =
{
  -0.000010686751596902973,
  0.0009921780986851866,
  0.000726541065473181,
  0.003328650972435994,
  0.0025516787456719286,
  0.007676165695618555,
  0.005410542756450859,
  0.013590997924082658,
  0.00813447737398761,
  0.019342941851670716,
  0.008442460492069767,
  0.022229886682445743,
  0.0038077362770273238,
  0.019852482780383915,
  -0.0069122715111913586,
  0.011877905367505165,
  -0.02185015047944249,
  0.001293955981732169,
  -0.03561872544896388,
  -0.005892674716983421,
  -0.04070569262288901,
  -0.002891603006615725,
  -0.030594453074971793,
  0.014444182934351313,
  -0.003309472464143412,
  0.04467682656024466,
  0.03649692553243876,
  0.08009634339132891,
  0.07834060876348199,
  0.10908428455204006,
  0.10962536265744334,
  0.12073638779210821,
  0.12073638779210821,
  0.10962536265744334,
  0.10908428455204006,
  0.07834060876348199,
  0.08009634339132891,
  0.03649692553243876,
  0.04467682656024466,
  -0.003309472464143412,
  0.014444182934351313,
  -0.030594453074971793,
  -0.002891603006615725,
  -0.04070569262288901,
  -0.005892674716983421,
  -0.03561872544896388,
  0.001293955981732169,
  -0.02185015047944249,
  0.011877905367505165,
  -0.0069122715111913586,
  0.019852482780383915,
  0.0038077362770273238,
  0.022229886682445743,
  0.008442460492069767,
  0.019342941851670716,
  0.00813447737398761,
  0.013590997924082658,
  0.005410542756450859,
  0.007676165695618555,
  0.0025516787456719286,
  0.003328650972435994,
  0.000726541065473181,
  0.0009921780986851866,
  -0.000010686751596902973
};
*/

/*
Better filter:

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 146484 Hz

* 0 Hz - 3000 Hz
  gain = 1
  desired ripple = 3 dB
  actual ripple = 0.0031900775460816854 dB

* 15000 Hz - 72000 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -97.3587719320948 dB

*/



float FIRcoefs[NUMFIRCOEFS] = {
  -0.000020205648087460712,
  -0.000038379492862792455,
  -0.00005851002711267839,
  -0.00006210573639865125,
  -0.000023548921581111526,
  0.00008909389734501491,
  0.00030612451419172843,
  0.0006439708479276814,
  0.0010899849881477357,
  0.001588290919137084,
  0.002031069938978739,
  0.0022609238360612213,
  0.002088081113387518,
  0.0013244390497096966,
  -0.00016819628024198827,
  -0.002422002440838275,
  -0.005310229485867333,
  -0.00851020061461215,
  -0.011495502652273731,
  -0.01356667230043595,
  -0.013924009775386166,
  -0.011777384841023145,
  -0.006479678892684055,
  0.0023373824618080365,
  0.014649664383441388,
  0.029973713836831204,
  0.04736489479525361,
  0.06548897491065975,
  0.08276245069579288,
  0.0975441691265355,
  0.10834984284900336,
  0.11405537275670001,
  0.11405537275670001,
  0.10834984284900336,
  0.0975441691265355,
  0.08276245069579288,
  0.06548897491065975,
  0.04736489479525361,
  0.029973713836831204,
  0.014649664383441388,
  0.0023373824618080365,
  -0.006479678892684055,
  -0.011777384841023145,
  -0.013924009775386166,
  -0.01356667230043595,
  -0.011495502652273731,
  -0.00851020061461215,
  -0.005310229485867333,
  -0.002422002440838275,
  -0.00016819628024198827,
  0.0013244390497096966,
  0.002088081113387518,
  0.0022609238360612213,
  0.002031069938978739,
  0.001588290919137084,
  0.0010899849881477357,
  0.0006439708479276814,
  0.00030612451419172843,
  0.00008909389734501491,
  -0.000023548921581111526,
  -0.00006210573639865125,
  -0.00005851002711267839,
  -0.000038379492862792455,
  -0.000020205648087460712
};





/*
float FIRcoefs[NUMFIRCOEFS] = 
{ 
0.00017789551662486736,
 0.00041040450128113212,
 0.00073564571611439266,
 0.0010288567261100756,
 0.0010932262399124643,
 0.00068102790709490837,
-0.0004289270055802503,
-0.0023236818651406581,
-0.0048507073861312269,
-0.0075593215529851923,
-0.0097324529011983477,
-0.010531001526389919,
-0.0092381283518636406,
-0.0055473909222043235,
 0.00019444056850832065,
 0.0068909705091059542,
 0.012822704762057578,
 0.016016481533045596,
 0.014801379079026093,
 0.0084175712824498886,
-0.0025017831741901836,
-0.015752677512255911,
-0.027802873657458295,
-0.034463933605740182,
-0.031871978947576575,
-0.017548806042311991,
 0.0087402114775642515,
 0.044592154434204508,
 0.085191505513169702,
 0.12415538783500346,
 0.15482622425492856,
 0.17171124432304663,
 0.17171124432304663,
 0.15482622425492856,
 0.12415538783500346,
 0.085191505513169702,
 0.044592154434204508,
 0.0087402114775642515,
-0.017548806042311991,
-0.031871978947576575,
-0.034463933605740182,
-0.027802873657458295,
-0.015752677512255911,
-0.0025017831741901836,
 0.0084175712824498886,
 0.014801379079026093,
 0.016016481533045596,
 0.012822704762057578,
 0.0068909705091059542,
 0.00019444056850832065,
-0.0055473909222043235,
-0.0092381283518636406,
-0.010531001526389919,
-0.0097324529011983477,
-0.0075593215529851923,
-0.0048507073861312269,
-0.0023236818651406581,
-0.0004289270055802503,
 0.00068102790709490837,
 0.0010932262399124643,
 0.0010288567261100756,
 0.00073564571611439266,
 0.00041040450128113212,
 0.00017789551662486736
 };
                    
*/
