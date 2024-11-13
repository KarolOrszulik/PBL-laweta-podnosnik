import matplotlib.pyplot as plt


def read_data(filename):
    data = []
    with open(filename) as f:
        for line in f:
            if 'real' in line:
                continue
            line = line.strip().split(',')
            data.append([float(line[0]), float(line[1])])
    return data


def linreg(data):
    Sx = sum([x[0] for x in data])
    Sy = sum([x[1] for x in data])
    Sxx = sum([x[0]**2 for x in data])
    Sxy = sum([x[0] * x[1] for x in data])
    n = len(data)

    a = (n * Sxy - Sx * Sy) / (n * Sxx - Sx**2)
    b = (Sy - a * Sx) / n

    y_mean = Sy / n
    SSres = sum([(x[1] - a * x[0] - b)**2 for x in data])
    SStot = sum([(x[1] - y_mean)**2 for x in data])
    R2 = 1 - SSres / SStot

    return a, b, R2


def write_output(filename, a, b, R2):
    with open(filename, 'w') as f:
        f.write(f'a = {a}\n')
        f.write(f'b = {b}\n')
        f.write(f'R2 = {R2}\n')


def plot_output(data, a, b):
    plt.scatter([x[0] for x in data], [x[1] for x in data])
    plt.plot([x[0] for x in data], [a * x[0] + b for x in data], color='red')
    plt.show()


def do_stuff(filename):
    data = read_data(filename)
    a, b, R2 = linreg(data)
    print(f'a = {a}, b = {b}, R2 = {R2}')
    write_output(filename.replace('.csv', '_output.txt'), a, b, R2)
    plot_output(data, a, b)


def main():
    # filenames = ['data_hx711_1.csv', 'data_hx711_2.csv', 'data_hx711_3.csv', 'data_hx711_4.csv']
    filenames = ['data_hx744_4.csv']

    for filename in filenames:
        do_stuff(filename)


if __name__ == '__main__':
    main()


