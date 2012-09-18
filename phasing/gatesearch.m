#function for calculating correct timer gating setting
#arguments: clks in one ofdm integration period, cycles of first timer, cycles of second timer
#returns: number of clks to set gate output of first timers low at end of each period, second timer period
function holdoff,reload=gatesearch(clks,cyclesone,cyclestwo)
	a=(clks.-([0:1:floor(clks/(cyclesone*4))].*cyclesone))./cyclestwo;
	b=find(abs(a.-round(a))<0.000001);
	reload=a(b(1));
	holdoff=b(1)-1;
endfunction
