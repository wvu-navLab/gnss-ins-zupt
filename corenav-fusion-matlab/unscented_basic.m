%The general formula is 2N+1, where N denotes the dimenstions
%N=1 => 3 Sigma points
% X^0=MeanOfGaussian
% X^i=MeanOfGaussian + (sqrt(n+Lambda)CovarianceMatrix)_i for i=1,...,n
% X^i=MeanOfGaussian - (sqrt(n+Lambda)CovarianceMatrix)_(i-n) for i=n+1,...,2n
% ? is the scaling factor which tells how much far from mean we should choose our sigma points. A good mathematical study suggests the optimal value of ? to be 3-n.

function[my, Py]=unscented_basic(mx,Px)
%mx Mean of vector x
%Px covariance of vector x
n=length(mx);
two_n=2*n;
nPs=chol(n*Px,'lower'); %Square root of Px
for i=1:n % calculate the sample points
    X{i}=mx+nPs(i,:);
    X{i+n}=mx-nPs(i,:);
end
for i=1:two_n %map the sample points
    Y{i}=map_x(X{i}); % Function map_x % y = 0.6/((1-chi0_slip));
end

my=zeros(1,n); % Calculate mean of y
for i=1:two_n
    my=my+Y{i};
end
my=my/two_n; %Mean of y
Py=zeros(n,n); % Calculate covariance of y
for i=1:two_n
e=Y{i}-my; % Row error vector
Py=Py+e*e';
end
Py=Py/ two_n/n; % Covariance of y
