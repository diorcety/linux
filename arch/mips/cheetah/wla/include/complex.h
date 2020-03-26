#ifndef __COMPLEX_H__
#define __COMPLEX_H__

typedef struct {
    double real, imag;
} complex;

complex complex_add(complex a, complex b) __attribute__ ((section(".rfc")));
complex complex_minus(complex a, complex b) __attribute__ ((section(".rfc")));
double complex_abs(complex a) __attribute__ ((section(".rfc")));
complex complex_multi(complex a, complex b) __attribute__ ((section(".rfc")));
complex complex_conj(complex a) __attribute__ ((section(".rfc")));
complex complex_div(complex a, complex b) __attribute__ ((section(".rfc")));
complex complex_mean(complex *arr, int arr_len) __attribute__ ((section(".rfc")));
double mean(double *arr, int arr_len) __attribute__ ((section(".rfc")));
int flt2hex(double x, int L) __attribute__ ((section(".rfc")));
double hex2flt(int x, int L) __attribute__ ((section(".rfc")));
double max_double(double a, double b) __attribute__ ((section(".rfc")));
double abs_double(double a) __attribute__ ((section(".rfc")));
double __complex_to_db(complex *x) __attribute__ ((section(".rfc")));
double abs_complex(complex *x) __attribute__ ((section(".rfc")));
double c_square(complex *c) __attribute__ ((section(".rfc")));

#endif //__COMPLEX_H__
