import matplotlib.pyplot as plt
import numpy as np
import os
import ast
import csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from advisor_client.client import *

font = {
    'family': 'monospace',
    'weight': 'normal',
    'size': 16,
}


def get_study(client, study_name):
    study = client.get_study_by_name(study_name)
    return study


def get_result(client, study_name):
    train_curve = []
    train_param = []
    trials = client.list_trials(study_name)

    for trial in trials:
        trial_metrics = client.list_trial_metrics(study_name, trial.id)
        train_param.append(ast.literal_eval(
            trial.parameter_values))  # string to dictionary
        for metrics in trial_metrics:
            train_curve.append(metrics.objective_value)

    for param, objval in zip(train_param, train_curve):
        param["objective_value"] = objval
        param.setdefault("jump_b", 0)
        param.setdefault("jump_c", 0)
        param.setdefault("jump_d", 0)

    train_curve = np.array(train_curve)

    return train_curve, train_param


def get_best_obj(train_curve):
    best_idx = np.argmin(train_curve)
    best_objval = round(train_curve[best_idx], 3)
    return best_idx, best_objval


def get_topk(train_curve, topk):
    return np.argsort(train_curve)[:topk]


# plot training objective value vs trails
def plot_obj(train_curve, train_param, save_fig=False):
    scale = 1000
    train_curve_clip = np.clip(train_curve, 0, scale)
    best_idx, best_objval = get_best_obj(train_curve)
    print("Is study done: %r" % client.is_study_done(study_name))
    print("Best epoch: %d" % best_idx)
    print("Best objval: %.3f" % best_objval)
    print("Best trial: {}".format(train_param[best_idx]))
    plt.style.use('seaborn')
    plt.plot(range(1, len(train_curve) + 1), train_curve_clip, linewidth=2)
    plt.xlabel("Trials", fontdict=font)
    plt.ylabel("Objective Values", fontdict=font)
    plt.xticks(fontsize=font["size"])
    plt.yticks(fontsize=font["size"])
    if save_fig:
        plt.savefig("../records/objval_" + str(best_objval) + "_" + str(scale) + ".png")
    plt.show()


def plot_torque(train_curve, train_param, topk=10, save_fig=False):
    topk_idx = get_topk(train_curve, topk)  # sort top k minimial index
    print("max objval: %.3f" % np.max(train_curve[topk_idx]))
    t = np.linspace(-0.1, 0.1, 100)
    y_constant = 35 * np.ones_like(t)
    for idx in topk_idx:
        param = train_param[idx]
        print(param)
        a = param["jump_a"]
        param.setdefault("jump_b", 0)
        param.setdefault("jump_c", 0)
        param.setdefault("jump_d", 0)
        # b = param["jump_b"]
        # c = param["jump_c"]
        # d = param["jump_d"]
        # y_poly = 10*a * t + 100*b * t**2 + 1000*c * t**3 + 10*d
        y_sigm = 35 / (1 + np.exp(-10 * a * (t)))
        plt.plot(t, y_sigm)
    plt.plot(t, y_constant)
    plt.xlabel("Time (sec)")
    plt.ylabel("Torque (Nm)")
    best_idx, best_objval = get_best_obj(train_curve)
    if save_fig:
        plt.savefig("../records/torque_" + str(best_objval) + ".png")
    plt.show()


# display top n trials in webots
def replay(train_curve, topk=5):
    topk_idx = get_topk(train_curve, topk)
    for count, idx in zip(range(topk), topk_idx):
        param_dic = train_param[idx]
        print(param_dic)
        param_dic.setdefault("jump_b", 0)
        param_dic.setdefault("jump_c", 0)
        param_dic.setdefault("jump_d", 0)
        with open("../controllers/my_controller_python/args.txt", 'w') as f11:
            f11.write(str(param_dic))

        os.system("webots --mode=pause")

        with open("../controllers/my_controller_python/metrics.txt", 'r') as f22:
            metrics_dic = eval(f22.read())
            y = metrics_dic["jump_metrics"]

        print("top %d, real_obj: %.3f, train_obj: %.3f" %
              (count, y, float(param_dic["objective_value"])))


def save_to_txt(train_param, name):
    # save dicts to txt
    with open(name, 'w') as arts_txt:
        for param_dict in train_param:
            arts_txt.write(str(param_dict) + "\n")  # one dictionary per line


# save dicts to csv
def save_to_csv(train_param, name):
    with open(name, 'w') as args_csv:
        w = csv.DictWriter(args_csv, train_param[0].keys())
        w.writeheader()
        for param_dict in train_param:
            w.writerow(param_dict)


# read from txt
def read_from_txt(name):
    train_param = []
    train_curve = []
    with open(name, 'r') as arts_txt:
        lines = arts_txt.readlines()
        for line in lines:
            train_param.append(eval(line))
            train_curve.append(train_param[-1]["objective_value"])
    train_curve = np.asarray(train_curve)
    return train_curve, train_param


# read from csv
def read_from_csv(name):
    train_param = []
    train_curve = []
    with open(name, 'r') as args_csv:
        dict_reader = csv.DictReader(args_csv)
        for param_dict in dict_reader:
            train_param.append(param_dict)
            train_curve.append(train_param[-1]["objective_value"])
    train_curve = np.asarray(train_curve)
    return train_curve, train_param


def design_sigmoid(a=5, init_torque=1, max_torque=35):
    """
    a           : rising speed of sigmoid
    init_torque : torque before jump
    """
    # 反解要拼接的起始点x坐标
    t_start = np.log(max_torque / init_torque - 1) / (-10 * a)
    print("t_start: %.3f" % t_start)
    tspan = np.linspace(t_start, 0.5, 200)
    y = max_torque / (1 + np.exp(-10 * a * (tspan)))
    plt.plot(tspan, y, linewidth=5)
    plt.axhline(color='k', linewidth=3)
    plt.axvline(color='k')
    plt.axhline(y=(max_torque) / 2, color='r')
    plt.axvline(x=0, color='r')
    plt.ylabel("torque")
    plt.xlabel("time")
    plt.show()


def plot_height_box(name, save_fig=False):
    df = pd.read_csv(name)
    # boxplot = df.boxplot(by ='height', column =['opt_vel'], grid = False)
    sns.set_style("darkgrid")
    # ax = sns.boxplot(data=df,x="height",y="opt_vel")
    data = np.asarray(df["opt_vel"])
    x_ticks = np.unique(np.asarray(df["height"]))
    h2_mean = data[:50].mean()
    h3_mean = data[50:100].mean()
    h4_mean = data[100:150].mean()
    values = [h2_mean, h3_mean, h4_mean]

    h2_std = data[:50].std()
    h3_std = data[50:100].std()
    h4_std = data[100:150].std()
    stds = [h2_std, h3_std, h4_std]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(np.arange(3),
           values,
           yerr=stds,
           align='center',
           ecolor='black',
           capsize=15,
           alpha=0.5)
    ax.set_ylabel('Optimal taking off speed (m/s)', fontdict=font)
    ax.set_xlabel("Desired jumping height (m)", fontdict=font)
    ax.set_xticks(np.arange(3))
    ax.tick_params(axis="y", labelsize=font["size"])
    ax.set_xticklabels(x_ticks, fontsize=font["size"])
    ax.set_ylim([2.2, 3.6])
    ax.yaxis.grid(True)
    if save_fig:
        plt.savefig("../records/opt_vel.png")
    plt.show()


if __name__ == "__main__":
    if not os.path.exists("../records"):
        os.mkdir("../records")
    # init Advisor client
    client = AdvisorClient()

    # fetch study
    study_name = "optimal_jump"
    study = get_study(study_name)
    print(study)

    # fetch optimization results
    train_curve, train_param = get_result(client, study_name)

    # plot
    plot_obj(train_curve, train_param)
    plot_torque(train_curve, train_param)

    # replay optimal result in webots
    replay(train_curve, 5)
    best_objval = get_best_obj(train_curve)

    # save results
    name_txt = "../records/args_" + str(best_objval) + ".txt"
    save_to_txt(train_param, name_txt)
    name_csv = "../records/args_" + str(best_objval) + ".csv"
    save_to_csv(train_param, name_csv)

    # once you have a jump library, you can plot this
    # name_box = "../records/boxplot.csv"
    # plot_height_box(name_box)
