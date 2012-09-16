function [i,j,k,peak]=phasesearch()
	for n=2700:1:2750
		for m=700:1:770
			for l=1120:1180
				peak(n-2699,m-699,l-1119)=max(abs(waves((n-2500)*pi/2500,(m-2500)*pi/2500,(l-2500)*pi/2500)));
			endfor
		endfor
	endfor
	min_=min(min(min(peak)));
	i,j,k=find(abs(min_-peak)<0.00001);
	k=ceil(j/size(peak)(2);
	j=mod(j,size(peak)(2));
endfunction
#this function tries to find the best starting phases for 5 carriers
#there are only 3 dof as one carrier is defined has zero phase, and one has zero due to time set
function waveform=waves(a,b,c)
	p=-pi:pi/200:pi;
	waveform=exp(i.*p).+exp(i.*(-p.+a)).+1.+exp(i.*(2.*p.+b)).+exp(i.*(-2.*p.+c));
endfunction

#for a=1:5000
#	dslice(a)=max(abs(waves_(a,ceil([1:1:5000^2]./5000),mod([1:1:5000^2],5000));
#endfor

#function waveform=waves_(a,b,c)
#	p=-pi:pi/200:pi;
#	waveform=exp(i.*p).+exp(i.*(-p.+(a-2500)*pi/2500)).+1.+exp(i.*(2.*p.+(b-2500)*pi/2500))).+exp(i.*(-2.*p.+(c-2500)*pi/2500));
#endfunction
