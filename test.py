# Description: program that processes a set of tasks in paralle
#
# Inspired by: https://docs.python.org/3/library/concurrent.futures.html#processpoolexecutor-example
#


import concurrent.futures
import math
import time
from vpython import vector

PRIMES = [
    112272535095293,
    112582705942171,
    112272535095293,
    115280095190773,
    115797848077099,
    1099726899285419]


def is_prime(n):
    if n < 2:
        return False
    if n == 2:
        return True
    if n % 2 == 0:
        return False
    v = vector(0,0,0)
    sqrt_n = int(math.floor(math.sqrt(n)))
    for i in range(3, sqrt_n + 1, 2):
        if n % i == 0:
            return False
    return True


def main():
    # Identify prime numbers without using parallel computation
    start_time = time.time()
    is_a_prime = [is_prime(cur_number) for cur_number in PRIMES]
    for number, prime in zip(PRIMES, is_a_prime):
        print('%d is prime: %s' % (number, prime))
    print("Linear computation finished task in {} seconds".format(time.time() - start_time))

    # Identify prime numbers with parallel computation
    start_time = time.time()
    with concurrent.futures.ProcessPoolExecutor() as executor:
        for number, prime in zip(PRIMES, executor.map(is_prime, PRIMES)):
            print('%d is prime: %s' % (number, prime))
    print("ProcessPoolExecutor finished task in {} seconds".format(time.time() - start_time))


if __name__ == '__main__':
    main()