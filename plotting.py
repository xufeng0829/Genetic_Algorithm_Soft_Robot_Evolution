import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


def learning_curve():
    data = pd.read_csv('learningcurve.csv')
    print(data.head())
    g = data['generation'].values
    d = data['distance'].values

    sns.set(font_scale=1.5)
    sns.set_style("whitegrid", {'grid.linestyle': '--'})
    sns.lineplot(x='generation', y='distance', ci=None, data=data, label='GA')

    plt.xlabel('generation')
    plt.ylabel('distance (m)')
    plt.show()


def dot_plot():
    data = pd.read_csv('dotplot.csv')
    print(data.head())
    g = data['generation'].values
    d = data['distance'].values

    sns.set(font_scale=1.5)
    sns.set_style("whitegrid", {'grid.linestyle': '--'})
    sns.scatterplot(x='generation', y='distance', data=data)

    plt.xlabel('generation')
    plt.ylabel('distance (m)')
    plt.show()


if __name__ == '__main__':
    dot_plot()
