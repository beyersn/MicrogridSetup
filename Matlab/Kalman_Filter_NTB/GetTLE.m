function [ longstr1,longstr2,rand_TLE] = GetTLE()

rand_TLE=randi(11);
% rand_TLE=3;

if rand_TLE==1
    %TLE 1
    longstr1 = '1 25544U 98067A   13213.51579142  .00005418  00000-0  10235-3 0  3046';
    longstr2 = '2 25544  51.6494 243.7352 0003674 255.5105 239.9716 15.50141065841738';

elseif rand_TLE==2
    %TLE 2: Estimated Orbit (TLE) (ISS)
    longstr1 = '1 00000U 15001A   13213.51579142  .00005418  00000-0  10235-3 0  3046';
    longstr2 = '2 00000  28.0000 247.4627 0290234 130.5360 325.0288 15.14248659000017';

elseif rand_TLE==3
    %TLE 3: Estimated Orbit (Oculus-ASR)
    longstr1 = '1 99999U          16122.70833333  .00000000  00000-0  00000-0 0 00000'; 
    longstr2 = '2 99999 028.5827 000.0589 0402629 358.9888 166.6245 15.00296682077477';

elseif rand_TLE==4
    %TLE 4:  Orbit Shape: Circular, Altitude: 500 km, Inclination: 45 deg, RAAN: 0 deg
    longstr1 = '1 99999U          15033.70833333 -.00000160  00000-0 -67666-5 0 00005';
    longstr2 = '2 99999 045.0200 000.0017 0007704 266.9920 093.0067 15.22469827000010'; 

elseif rand_TLE==5
    %TLE 5:  Orbit Shape: Circular, Altitude: 400 km, Inclination: 28.6 deg, RAAN: 0 deg
    longstr1 = '1 99999U          15033.70833333 -.00001332  00000-0 -17539-4 0 00000';
    longstr2 = '2 99999 028.6173 000.0189 0005341 264.4977 095.4861 15.57216909000013';

elseif rand_TLE==6
    %TLE 6:  Orbit Shape: Elliptical, Apogee: 844 km,  Perigee: 300 km, Inclination: 28.6 deg, RAAN: 0 deg
    longstr1 = '1 99999U          15033.70833333  .00000308  00000-0  49309-5 0 00001';
    longstr2 = '2 99999 028.6071 283.4076 0389642 354.0032 003.1775 14.97045727000010';

elseif rand_TLE==7
    %TLE 7:  Orbit Shape: Circular, Altitude: 700 km, Inclination: 50 deg, RAAN: 15 deg
    longstr1 = '1 99999U          15033.70833333 -.00000084  00000-0 -17992-4 0 00006';
    longstr2 = '2 99999 050.0186 014.9980 0008106 268.1847 091.8161 14.58119731000016';

elseif rand_TLE==8
    %TLE 8:  Orbit Shape: Circular, Altitude: 300 km, Inclination: 18 deg, RAAN: 18 deg
    longstr1 = '1 99999U          15033.70833333  .00000626  00000-0  16914-5 0 00000';
    longstr2 = '2 99999 018.0125 018.0316 0003456 262.4266 097.5453 15.92844770000010';

elseif rand_TLE==9
    %TLE 9:  Orbit Shape: Circular, Altitude: 600 km, Inclination: 36 deg, RAAN: 89 deg
    longstr1 = '1 99999U          15033.70833333  .00000692  00000-0  69651-4 0 00008';
    longstr2 = '2 99999 036.0185 089.0090 0006300 265.8723 094.1213 14.90315331000019';

elseif rand_TLE==10
    % TLE 10:  Orbit Shape: Circular, Altitude: 800 km, Inclination: 45 deg, RAAN: 56 deg
    longstr1 = '1 99999U          15033.70833333 -.00000120  00000-0 -47806-4 0 00004';
    longstr2 = '2 99999 045.0183 056.0010 0007389 267.4023 092.5969 14.27988745000017';

elseif rand_TLE==11
    % TLE 11:  Orbit Shape: Circular, Altitude: 900 km, Inclination: 26 deg, RAAN: 37 deg
    longstr1 = '1 99999U          15033.70833333  .00000614  00000-0  41452-3 0 00000';
    longstr2 = '2 99999 026.0141 037.0141 0004505 264.9931 094.9957 13.99455367000011';

end

end

