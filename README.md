# BlastGateControl
Arduino code for making automatic blast gates in my workshop.

This program is inspired by the work of Bob Clagett from I Like To Make Stuff https://github.com/iliketomakestuff/iltms_automated_dust_collection/blob/master/DustCollectionAutomation_v2.ino

It's heavily modified, because the AC current sensor he used, does not work well with 230V. I have replaced the sensors with two ADS1115 and current clamps, which gives me control over a total of 4 blast gates. I also have an additional blast gate controled with a push button. It's possible to hook up another 2 ADS115 so the total number of current-controlled blast gates can be 8.