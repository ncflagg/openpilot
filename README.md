openpilot 0.6 Prius Prime INDI tune
======

An attempt to create a laterally-solid and stable build for the Prius Prime. Pretty solid experience at the moment if I do say so myself. No ***ping-pong*** to speak of, even with the stock angle/torque sensor.

40-45mph video, testing (play it at 2x): - http://www.youtube.com/watch?v=lthQ-q9AKCg?vq=hd720

[![](http://img.youtube.com/vi/lthQ-q9AKCg/0.jpg)](http://www.youtube.com/watch?v=lthQ-q9AKCg?vq=hd720 "40-45mph video, testing")

40-45mph night/wind storm version (play it at 2x) - https://www.youtube.com/watch?v=Kh398yP7rBQ?vq=hd720

[![](http://img.youtube.com/vi/Kh398yP7rBQ/0.jpg)](https://www.youtube.com/watch?v=Kh398yP7rBQ?vq=hd720 "40-45mph night/wind storm version")

After incorporating a lane width mod from Gernby, plus some of my other tuning, I had a drive good enough to warrant saving that work. I don't understand yet which mods are contributing to the good lateral performance.

#### Theses changes are what I saw the most immediate and obvious benefit:
---
 - Zorro's / Gernby's lane width mod; one (and maybe another) version in particular
 - `actuatorEffectiveness` to 1.2 smoothed some of the wheel jerking
 - `actuatorDelay` to 0.25 to eliminate certain hugging (not the double-yellow recognition)
 - Having a stable `angleOffsetAverage`;  watch it closely in `LiveParameters`. It should stay close to what you have now, to avoid ping pong
 - `steerRateCost` up
 - `timeConstant` up

#### Issues
 - Some *opposite* lane hugging; meaning, where stock OP might hug the inside of a turn, all the hugging I see (and it's much more rare) is at the outside of the lane. I'm ready to move on to 0.6.2 now and see what replicates and see if the hugging can be eliminated.
 - **Vehicle Parameter Identification Failed** messages on EON while engaged. May be due to the params_learner being crippled. If it's true I'm not really using the fast ao learner at all anyway, might just disable it

<br/>
I've made a couple attempts to revert some of my changes one-by-one in order to see which ones were really improving things and nearly lost my work, hence, this saved snapshot to come back to.

All learners in params_learner have been disabled except for the fast offset learner which, I'm currently thinking means nothing in the learner is being used. Actually, `angleOffsetAverage` is still enabled but very slowly.
 
 My `/data/params/d/LiveParameters` looks something like below, as should yours (other than maybe the angleOffsetAverage):
 
`{"angleOffsetAverage": -1.2, "carFingerprint": "TOYOTA PRIUS 2017", "carVin": "", "steerRatio": 15.0, "stiffnessFactor": 1}`

Backup your existing params with:

`cp /data/params/d/LiveParameters /data/params/d/LiveParameters.orig`
