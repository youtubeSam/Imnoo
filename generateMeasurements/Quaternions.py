#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

def qMult(q, r):
  """Product of two quaternions.
  input
  q = q_0 + i*q_1 + j*q_2 + k*q_3
  r = r_0 + i*r_1 + j*r_2 + k*r_3
  returns q x r
  """
  n = [0, 0, 0, 0]
  n[0] = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3]
  n[1] = q[1] * r[0] + q[0] * r[1] - q[3] * r[2] + q[2] * r[3]
  n[2] = q[2] * r[0] + q[3] * r[1] + q[0] * r[2] - q[1] * r[3]
  n[3] = q[3] * r[0] - q[2] * r[1] + q[1] * r[2] + q[0] * r[3]
  return n

def qInv(q):
  """Inverse of quaternion.
  input
  q = q_0 + i*q_1 + j*q_2 + k*q_3
  returns q^-1
  """
  n = [0, 0, 0, 0]
  denom = 1 / (q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
  n[0] = denom * q[0]
  n[1] = -denom * q[1]
  n[2] = -denom * q[2]
  n[3] = -denom * q[3]
  return n


def qConj(q):
  """Conjugate quaternion
  input
  q = q_0 + i*q_1 + j*q_2 + k*q_3
  return q*
  """
  n = [0, 0, 0, 0]
  n[0] = q[0]
  n[1] = -q[1]
  n[2] = -q[2]
  n[3] = -q[3]
  return n


def quadToRot(q):
  I = np.identity(4)
  R = np.zeros((3, 3))
  for i in range(3):
    q_r = np.array(qMult(qMult(q, I[:, i + 1]), qConj(q)))
    R[:, i] = (q_r[1:4])
  return R
