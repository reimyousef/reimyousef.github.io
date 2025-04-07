// --------------------------------------------------
// This C program implements content-aware image resizing using seam carving.
// It calculates pixel energy, uses dynamic programming to find the lowest-energy seam,
// then removes that seam to reduce image width while preserving important content.
//
// Key Functions:
// - calc_energy: Computes the energy of each pixel (based on RGB gradient).
// - dynamic_seam: Builds DP table of cumulative minimum energies.
// - recover_path: Extracts the optimal seam path.
// - remove_seam: Removes the seam from the image.
//
// Dependencies: c_img.h, seamcarving.h (custom image manipulation headers)

#include "c_img.h"
#include "seamcarving.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

void calc_energy(struct rgb_img *im, struct rgb_img **grad) {
    create_img(grad, im->height, im->width);
    for (int y = 0; y < im->height; y++) {
        for (int x = 0; x < im->width; x++) {
            int rx, ry, gx, gy, bx, by;

            // Handle edge and corner cases
            // Compute gradients in x and y using wrapped indices (circular)
            int up = (y == 0) ? im->height - 1 : y - 1;
            int down = (y == im->height - 1) ? 0 : y + 1;
            int left = (x == 0) ? im->width - 1 : x - 1;
            int right = (x == im->width - 1) ? 0 : x + 1;

            ry = get_pixel(im, down, x, 0) - get_pixel(im, up, x, 0);
            gy = get_pixel(im, down, x, 1) - get_pixel(im, up, x, 1);
            by = get_pixel(im, down, x, 2) - get_pixel(im, up, x, 2);

            rx = get_pixel(im, y, right, 0) - get_pixel(im, y, left, 0);
            gx = get_pixel(im, y, right, 1) - get_pixel(im, y, left, 1);
            bx = get_pixel(im, y, right, 2) - get_pixel(im, y, left, 2);

            int delta_x = rx*rx + gx*gx + bx*bx;
            int delta_y = ry*ry + gy*gy + by*by;
            uint8_t energy = (uint8_t)(sqrt(delta_x + delta_y) / 10);

            set_pixel(*grad, y, x, energy, energy, energy);
        }
    }
}

void dynamic_seam(struct rgb_img *grad, double **best_arr) {
    *best_arr = (double *)malloc(sizeof(double) * grad->width * grad->height);

    for (int i = 0; i < grad->height; i++) {
        for (int j = 0; j < grad->width; j++) {
            double e = (double)get_pixel(grad, i, j, 0);
            if (i == 0) {
                (*best_arr)[i * grad->width + j] = e;
            } else {
                double min_val = (*best_arr)[(i-1)*grad->width + j];
                if (j > 0) min_val = fmin(min_val, (*best_arr)[(i-1)*grad->width + j - 1]);
                if (j < grad->width - 1) min_val = fmin(min_val, (*best_arr)[(i-1)*grad->width + j + 1]);
                (*best_arr)[i * grad->width + j] = e + min_val;
            }
        }
    }
}

void recover_path(double *best, int height, int width, int **path) {
    *path = malloc(sizeof(int) * height);
    int min_idx = 0;

    // Find index of minimum energy in last row
    double min_val = best[(height - 1) * width];
    for (int j = 1; j < width; j++) {
        double val = best[(height - 1) * width + j];
        if (val < min_val) {
            min_val = val;
            min_idx = j;
        }
    }

    (*path)[height - 1] = min_idx;

    // Backtrack
    for (int i = height - 2; i >= 0; i--) {
        int prev = (*path)[i + 1];
        int start = fmax(0, prev - 1);
        int end = fmin(width - 1, prev + 1);
        double min_val = best[i * width + start];
        int best_idx = start;
        for (int j = start + 1; j <= end; j++) {
            if (best[i * width + j] < min_val) {
                min_val = best[i * width + j];
                best_idx = j;
            }
        }
        (*path)[i] = best_idx;
    }
}

void remove_seam(struct rgb_img *src, struct rgb_img **dest, int *path) {
    create_img(dest, src->height, src->width - 1);
    for (int i = 0; i < src->height; i++) {
        int skip = path[i];
        for (int j = 0; j < src->width - 1; j++) {
            int from_j = j < skip ? j : j + 1;
            uint8_t r = get_pixel(src, i, from_j, 0);
            uint8_t g = get_pixel(src, i, from_j, 1);
            uint8_t b = get_pixel(src, i, from_j, 2);
            set_pixel(*dest, i, j, r, g, b);
        }
    }
}

// Optional: Example usage can be put in a separate test.c file or commented main
// to demonstrate a seam removal pipeline step-by-step.