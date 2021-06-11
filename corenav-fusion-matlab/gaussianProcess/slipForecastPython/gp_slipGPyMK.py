import GPy
from matplotlib import pyplot as plt
import numpy as np
import scipy.io as sio
from utils.data import *
#DIR='C:\\Users\\Cagri Kilic\\Desktop\\pathfinderFast\\pathfinder_test_2019-08-25\\scenario2_long\\scripts\\cagriGPcodes\\slipForecastPython'
DIR='C:\\Users\\k_cag\Desktop\corenav-GP-matlab\\gaussianProcess\\slipForecastPython'
data = load_slip(DIR+"/slipVal.csv")
X = np.array(range(1,len(data)+1)).reshape(len(data),1)
Y = np.array(data).reshape(len(data),1)
per = 0.9
x_train, y_train = X[:int(per*len(X))], Y[:int(per*len(Y))]
x_test, y_test = X[int(per*len(X)):], Y[int(per*len(Y)):]
# kerns = [GPy.kern.RBF(1), GPy.kern.Exponential(1),
# GPy.kern.Matern32(1), GPy.kern.Matern52(1), GPy.kern.Brownian(1),
# GPy.kern.Bias(1), GPy.kern.Linear(1), GPy.kern.PeriodicExponential(1),
# GPy.kern.White(1)]


# kernel = GPy.kern.RBF(1) + GPy.kern.Linear(1) +GPy.kern.Brownian(1)# Equivalent to kernel = GPy.kern.rbf(input_dim=1, variance=1., lengthscale=1.)
kernel=(GPy.kern.RBF(1))*GPy.kern.Brownian(1)
# kernel =GPy.kern.RBF(1)* GPy.kern.Linear(1)
m = GPy.models.GPRegression(x_train,y_train,kernel)
m.optimize(messages=False)

from IPython.display import display
display(m)
means= [] #predictions
variances = [] #uncertainty
# X_=np.arange(X.min(),X.max()+50, 0.1)
X_=np.arange(X.min(),X.max()+600, 1)

for x in X_:
    mean,covar = m.predict(np.array([[x]]))
    variances.append(covar[0])
    means.append(mean[0])

ses = 2*np.sqrt(np.array(variances))
means = np.array(means)
var1=np.array(means+ses).reshape(1,len(means+ses))
var2=np.array(means-ses).reshape(1,len(means-ses))
# print(means.reshape(1,len(means)))
#
# print(ses.reshape(1,len(means)))
m.plot()
plt.xlim(X.min(), X_.max())
plt.plot(X,Y, c='r',linewidth=0.2)
plt.fill_between(X_, var2[0,:], var1[0,:], alpha=0.3, color='k') # confidence intervals #C0C0C0
plt.plot(x_test, y_test,'xk')
plt.ylim(-1.0, 1.0)
plt.show()
# means=means.reshape(1,len(means))
# ses=ses.reshape(1,len(means))
sio.savemat(DIR+'/means.mat', {'means':means.reshape(1,len(means))})
sio.savemat(DIR+'/ses.mat', {'ses':ses.reshape(1,len(means))})
