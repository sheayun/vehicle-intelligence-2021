import numpy as np

def gen(N, mu, sigma):
    v = np.random.normal(mu, sigma, N)
    return v

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 4:
        print("Usage: %s <N> <m> <s>" % sys.argv[0])
        sys.exit(0)

    n = int(sys.argv[1])
    m = int(sys.argv[2])
    s = int(sys.argv[3])

    r = gen(n, 0, s)

    for x in r:
        print("\t%e" % x)
