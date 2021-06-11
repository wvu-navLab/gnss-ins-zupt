import pandas as pd


def load_co2(file_path):
    co2 = pd.read_csv(file_path, header=None, index_col=None)

    co2_x = co2[1].values
    co2_y = co2[0].values

    return co2_x, co2_y


def load_erie(file_path):
    erie = pd.read_csv(file_path, header=0, names=["date", "level"], index_col=None)

    erie = erie["level"].values
    erie = erie[:len(erie)-1]
    return erie

def load_slip(file_path):
    slip = pd.read_csv(file_path, header=0, names=["date", "level"], index_col=None)

    slip = slip["level"].values
    slip = slip[:len(slip)-1]
    return slip

def load_airline(file_path):
    airline = pd.read_csv(file_path, header=None)
    airline = airline[1].values

    return airline


def load_solar(file_path):
    solar = pd.read_csv(file_path, sep='     ', header=None, skiprows=7)
    solar = solar[1].values

    return solar
