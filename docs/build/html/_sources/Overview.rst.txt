
Overview
--------

**Factor graphs** are graphical models (`Koller and Friedman,
2009 <#LyXCite-Koller09book>`__) that are well suited to modeling
complex estimation problems, such as Simultaneous Localization and
Mapping (SLAM) or Structure from Motion (SFM). You might be familiar
with another often used graphical model, Bayes networks, which are
directed acyclic graphs. A **factor graph,** however, is a *bipartite*
graph consisting of factors connected to variables. The **variables**
represent the unknown random variables in the estimation problem,
whereas the **factors** represent probabilistic constraints on those
variables, derived from measurements or prior knowledge. In the
following sections I will illustrate this with examples from both
robotics and vision.

The GTSAM toolbox (GTSAM stands for “Georgia Tech Smoothing and
Mapping”) toolbox is a BSD-licensed C++ library based on factor graphs,
developed at the Georgia Institute of Technology by myself, many of my
students, and collaborators. It provides state of the art solutions to
the SLAM and SFM problems, but can also be used to model and solve both
simpler and more complex estimation problems. It also provides a MATLAB
interface which allows for rapid prototype development, visualization,
and user interaction.

GTSAM exploits sparsity to be computationally efficient. Typically
measurements only provide information on the relationship between a
handful of variables, and hence the resulting factor graph will be
sparsely connected. This is exploited by the algorithms implemented in
GTSAM to reduce computational complexity. Even when graphs are too dense
to be handled efficiently by direct methods, GTSAM provides iterative
methods that are quite efficient regardless.

You can download the latest version of GTSAM from our `Github
repo <https://github.com/borglab/gtsam>`__.


Acknowledgements
~~~~~~~~~~~~~~~~~

GTSAM was made possible by the efforts of many collaborators at Georgia
Tech and elsewhere, including but not limited to Doru Balcan, Chris
Beall, Alex Cunningham, Alireza Fathi, Eohan George, Viorela Ila,
Yong-Dian Jian, Michael Kaess, Kai Ni, Carlos Nieto, Duy-Nguyen Ta,
Manohar Paluri, Christian Potthast, Richard Roberts, Grant Schindler,
and Stephen Williams. In addition, Paritosh Mohan helped me with the
manual. Many thanks all for your hard work!
