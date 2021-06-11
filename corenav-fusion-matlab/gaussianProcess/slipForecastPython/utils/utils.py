import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import MinMaxScaler


def scale(x_train, y_train, x_test, y_test):
    scaler = MinMaxScaler(feature_range=(-1, 1))

    scale_values = np.concatenate((x_train.reshape(-1), y_train.reshape(-1)))
    scale_values = np.unique(scale_values)
    scaler.fit(scale_values.reshape(-1, 1))

    x_train = scaler.transform(x_train.reshape(-1, 1)).reshape(x_train.shape[0], x_train.shape[1], x_train.shape[2])
    y_train = scaler.transform(y_train.reshape(-1, 1)).reshape(y_train.shape[0], y_train.shape[1])

    x_test = scaler.transform(x_test.reshape(-1, 1)).reshape(x_test.shape[0], x_test.shape[1], x_test.shape[2])
    y_test = scaler.transform(y_test.reshape(-1, 1)).reshape(y_test.shape[0], y_test.shape[1])

    return scaler, x_train, y_train, x_test, y_test


def detrend(data, train_percent, window_length):
    idx = int(len(data) * train_percent) - (window_length - 1)

    model = LinearRegression()
    model.fit(np.arange(idx).reshape(-1, 1), data[:idx])

    trend = model.predict(np.arange(len(data)).reshape(-1, 1))

    data = data - trend

    return data, trend


def create_window_data(data, window_size):
    temp = []

    for i in range(len(data) - window_size):
        temp.append(data[i: i + window_size])

    temp = np.array(temp)

    x = temp[:, :-1]
    y = temp[:, -1]

    x = np.expand_dims(x, axis=2)
    y = np.expand_dims(y, axis=1)

    return x, y


def create_direct_data(data, window_length, train_percent):
    data, trend = detrend(data, train_percent, window_length)

    x, y = create_window_data(data, window_length + 1)
    split = int(train_percent * len(x))

    x_train, y_train = x[:split], y[:split]
    x_test, y_test = x[split:], y[split:]

    scaler, x_train, y_train, x_test, y_test = scale(x_train, y_train, x_test, y_test)

    return x_train, y_train, x_test, y_test, scaler, trend


def predict_sliding(model, x_train, test_len, seq_len, window_length, full=True):
    train_pred = model.predict(x_train)
    cur_window = train_pred[-seq_len:]
    cur_window = cur_window.reshape(window_length)
    pred = []

    for i in range(test_len):
        pred.append(model.predict(np.expand_dims(cur_window, axis=0))[0][0])
        cur_window = cur_window[1:]
        cur_window = np.concatenate((cur_window, np.array([pred[i]])))

    pred = np.array(pred)

    if full:
        result = np.concatenate((train_pred.reshape(-1), pred))
    else:
        result = pred

    return result


def predict_sliding_rnn(model, x_train, test_len, seq_len, window_length, full=True):
    train_pred = model.predict(x_train)
    cur_window = train_pred[-seq_len:]
    cur_window = cur_window.reshape(window_length)
    pred = []

    for i in range(test_len):
        pred.append(model.predict(np.expand_dims(cur_window, axis=0))[0][0])
        cur_window = cur_window[1:]
        cur_window = np.concatenate((cur_window, np.array([pred[i]])))

    pred = np.array(pred)

    if full:
        result = np.concatenate((train_pred.reshape(-1), pred))
    else:
        result = pred

    return result


def invert_scale(scaler, values):
    if values.ndim == 1:
        values = np.reshape(values, (-1, 1))

    inverted = scaler.inverse_transform(values)
    inverted = np.reshape(inverted, -1)

    return inverted
