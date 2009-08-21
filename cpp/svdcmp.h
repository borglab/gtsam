/**
 * @file    svdcmp.h
 * @brief   SVD decomposition adapted from NRC
 * @author  Alireza Fathi
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once

/** SVD decomposition */
void svdcmp(double **a, int m, int n, double w[], double **v);

