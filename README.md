openpilot 0.6 Prius Prime INDI tune
======

An attempt to create a laterally-solid and stable build for the Prius Prime

40-45mph video, testing (play it at 2x):
[![](http://img.youtube.com/vi/lthQ-q9AKCg/0.jpg)](http://www.youtube.com/watch?v=lthQ-q9AKCg "40-45mph video, testing last night (play it at 2x)")

40-45mph night/wind storm version  (play it at 2x)
https://www.youtube.com/watch?v=Kh398yP7rBQ?vq=hd720

After incorporating a lane width mod from Gernby, plus some of my other tuning, I had a drive good enough to warrant saving that work. I don't understand yet which mods are contributing to the good lateral performance.

Theses changes are what I saw the most immediate and obvious benefit:

 -Gernby's lane width mod; one (and maybe another) version in particular

-Bumping up actuator effectiveness to 1.2
 
 
 I've made a couple attempts to revert some of my changes one-by-one in order to see which ones were really improving things and nearly lost my work, hence, this saved snapshot to come back to.
 
 Something I've done with tire stiffness is causing the value in LiveParameters to never change. I don't consider this a bad thing. The angle offset in LiveParameters still changes too much for my taste so I'll try retarding that further in the future.
 
 My /data/params/d/LiveParameters looks something like below, as should yours (other than may the angleOffsetAverage):
   {"angleOffsetAverage": -1.2, "carFingerprint": "TOYOTA PRIUS 2017", "carVin": "", "steerRatio": 16.102018271985425, "stiffnessFactor": 1
