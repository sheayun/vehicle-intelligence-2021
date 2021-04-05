import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = (12, 8)

def plot_2d(estimations, ground_truth):
    plt.cla()

    gt_x = [g['px'] for g in ground_truth]
    gt_y = [g['py'] for g in ground_truth]
    es_x = [e['px'] for e in estimations]
    es_y = [e['py'] for e in estimations]

    plt.scatter(gt_x, gt_y, s=1, label='Ground Truth')
    plt.scatter(es_x, es_y, s=1, label='Estimation')
    plt.legend()

    plt.gcf().canvas.set_window_title('EKF Example')
    plt.show()