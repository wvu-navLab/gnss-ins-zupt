import matplotlib.pyplot as plt
import numpy as np


def plot_co2(y_true, mean, lb, ub, trainlen, n, r):
    plt.plot(range(len(y_true)),y_true,'b',label="Actual")
    plt.plot(range(len(y_true)),mean,'r', label="ESN Prediction")
    plt.fill_between(range(len(y_true)), lb, ub, facecolor='grey', alpha=0.3)
    lo,hi = plt.ylim()
    plt.plot([trainlen,trainlen],[lo+np.spacing(1),hi-np.spacing(1)],'k:')
    plt.xlabel('Months since Aug 1960')
    plt.ylabel('CO2 Concentration (ppmv)')
    plt.legend(loc=2)
    plt.show()


def plot_erie(y_true, mean, lb, ub, trainlen, n, r):
    plt.plot(range(len(y_true)),y_true,'b',label="Actual")
    plt.plot(range(len(y_true)),mean,'r', label="ESN Prediction")
    plt.fill_between(range(len(y_true)), lb, ub, facecolor='grey', alpha=0.3)
    lo,hi = plt.ylim()
    plt.plot([trainlen,trainlen],[lo+np.spacing(1),hi-np.spacing(1)],'k:')
    plt.xlabel('Months since Aug 1922')
    plt.ylabel('Water Level')
    plt.legend(loc=2)
    plt.show()

def plot_airline(y_true, mean, lb, ub, trainlen, n, r):
    plt.plot(range(len(y_true)),y_true,'b',label="Target")
    plt.plot(range(len(y_true)),mean,'r', label="ESN n="+str(n)+", r="+str(r))
    plt.fill_between(range(len(y_true)), lb, ub, facecolor='grey', alpha=0.3)
    lo,hi = plt.ylim()
    plt.plot([trainlen,trainlen],[lo+np.spacing(1),hi-np.spacing(1)],'k:')
    plt.xlabel('Time (months)')
    plt.ylabel('Number of Passengers')
    plt.legend(loc=2, fontsize='x-small')
    plt.show()


def plot_solar(y_true, mean, lb, ub, trainlen, n, r):
    plt.plot(range(len(y_true)),y_true,'b',label="Target")
    plt.plot(range(len(y_true)),mean,'r', label="ESN n="+str(n)+", r="+str(r))
    plt.fill_between(range(len(y_true)), lb, ub, facecolor='grey', alpha=0.3)
    lo,hi = plt.ylim()
    plt.plot([trainlen,trainlen],[lo+np.spacing(1),hi-np.spacing(1)],'k:')
    plt.xlabel('Years since 1629')
    plt.ylabel('TSI (w/m^2)')
    plt.legend(loc=2, fontsize='x-small')
    plt.show()
