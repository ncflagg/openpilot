openpilot 0.6 Prius Prime INDI tune
======

An attempt to create a laterally-solid and stable build for the Prius Prime

After incorporating a lane width mod from Gernby, plus some of my other tuning, I had a drive good enough to warrant saving that work. I don't understand yet which mods are contributing to the good lateral performance.

Theses changes are what I saw the most immediate and obvious benefit:

 -Gernby's lane width mod; one (and maybe another) version in particular

-Bumping up actuator effectiveness to 1.2
 
 I've made a couple attempts to pull some of my changes one-by-one and nearly lost my work, hence, this saved snapshot to come back to.
 
 Something I've done with tire stiffness is causing the value in LiveParameters to never change. I don't consider this a bad thing. The angle offset in LiveParameters still changes too much for my taste so I'll try retarding that further in the future.
 
 My /data/params/d/LiveParameters looks something like below, as should yours (other than may the angleOffsetAverage):
   {"angleOffsetAverage": -1.2, "carFingerprint": "TOYOTA PRIUS 2017", "carVin": "", "steerRatio": 16.102018271985425, "stiffnessFactor": 1
