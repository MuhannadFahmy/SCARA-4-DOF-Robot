syms t;
theta(t)= 10 + 5*t + 70*t^2 - 45*t^3;
t = [0 1];
subs(theta(t))

Df = diff(theta);

Df0 = Df(0)
Df1 = Df(1)

DDf = diff(Df)

DDf0 = DDf(0)
DDf1 = DDf(1)
