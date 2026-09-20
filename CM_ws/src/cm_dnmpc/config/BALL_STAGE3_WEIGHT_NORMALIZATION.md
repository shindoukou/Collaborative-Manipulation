# Ball Stage 3 weight normalization

The Stage 3 objective uses dimensionless residuals:

```text
J = sum_j W_j * ||e_j / s_j||^2
```

`stage_weights.ball.3` stores the dimensionless weights `W_j` and
`stage_weight_error_scales.ball.3` stores the physical scales `s_j`.
The selected Ball Stage 3 values are:

| term | weight | physical scale |
|---|---:|---:|
| `Wd` | 1 | 0.010 m |
| `Wt` | 100 | 0.0075 m |
| `Wo` | 25 | 0.05 direction-vector units |
| `Wu` | 0.1 | 0.1 rad/s |
| `Wq` | 25 | 0.05 rad |
| `Wv` | 1 | 0.1 rad/s |
| `WF` | 100 | 5 N |

These values were selected on one frozen set of 20 feasible Ball Move
references.  The x1 setting achieved 20/20 controller success and 20/20
collision-free post-validation using the Gazebo-equivalent robot, obstacle,
and carried-ball geometry.  Collision checking remains a post-validation step;
it is not part of the DNMPC objective.
