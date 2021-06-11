import pandas as pd


def read_mauna_load_data():
    co2_dataset = pd.read_csv('mauna-loa-atmospheric-co2.csv', header=None)
    co2_dataset.columns = ['CO2Concentration', 'Time']

    return co2_dataset


def read_airline_data():
    airline_dataset = pd.read_csv('airline_data.csv', header=None)
    airline_dataset.columns = ['Time', 'Passengers']

    return airline_dataset


def read_solar_irradiation_data():
    solar_dataset = pd.read_csv('solar_irradiance.txt', sep='     ',
                                header=None, skiprows=7)
    solar_dataset.columns = ['Year', 'TSI']

    return solar_dataset
