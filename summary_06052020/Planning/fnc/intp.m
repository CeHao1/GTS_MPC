function out=intp(x,K)
t=1:1:length(x);
% k=0:(max(t)-min(t))/K:max(t);
k=(1:K)/K*length(x);
out=spline (t,x,k);


end