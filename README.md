openpilot 0.6 Prius Prime INDI tune
======

An attempt to create a laterally-solid and stable build for the Prius Prime

40-45mph video, testing (play it at 2x): - http://www.youtube.com/watch?v=lthQ-q9AKCg?vq=hd720

[![](http://img.youtube.com/vi/lthQ-q9AKCg/0.jpg)](http://www.youtube.com/watch?v=lthQ-q9AKCg?vq=hd720 "40-45mph video, testing")

40-45mph night/wind storm version (play it at 2x) - https://www.youtube.com/watch?v=Kh398yP7rBQ?vq=hd720

[![](http://img.youtube.com/vi/Kh398yP7rBQ/0.jpg)](https://www.youtube.com/watch?v=Kh398yP7rBQ?vq=hd720 "40-45mph night/wind storm version")

After incorporating a lane width mod from Gernby, plus some of my other tuning, I had a drive good enough to warrant saving that work. I don't understand yet which mods are contributing to the good lateral performance.

Theses changes are what I saw the most immediate and obvious benefit:

 -Gernby's lane width mod; one (and maybe another) version in particular

 -Bumping up actuator effectiveness to 1.2
 
 -ActuatorDelay to 0.25 to eliminate certain hugging (not the double-yellow recognition)
 
 -Watch angleOffsetAverage closely in `LiveParameters`. It should stay close to what you have now, to avoid ping pong
 
 
 I've made a couple attempts to revert some of my changes one-by-one in order to see which ones were really improving things and nearly lost my work, hence, this saved snapshot to come back to.
 
 Something I've done with tire stiffness is causing the value in LiveParameters to never change. I don't consider this a bad thing. The angle offset in LiveParameters still changes too much for my taste so I'll try retarding that further in the future.
 
 My /data/params/d/LiveParameters looks something like below, as should yours (other than may the angleOffsetAverage):
 
`{"angleOffsetAverage": -1.2, "carFingerprint": "TOYOTA PRIUS 2017", "carVin": "", "steerRatio": 16.1, "stiffnessFactor": 1}`

Backup your existing params with:

`cp /data/params/d/LiveParameters /data/params/d/LiveParameters.orig`
