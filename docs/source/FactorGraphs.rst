Factor Graphs
---------------

Let us start with a one-page primer on factor graphs, which in no way
replaces the excellent and detailed reviews by `Kschischang et
al. <#LyXCite-Kschischang01it>`__ (2001) and
`Loeliger <#LyXCite-Loeliger04spm>`__ (2004).


|image: 2\_Users\_dellaert\_git\_github\_doc\_images\_hmm.png| Figure 1:
An HMM, unrolled over three time-steps, represented by a Bayes net.

Figure `1 <#fig_unrolledHMM>`__ shows the **Bayes network** for a hidden
Markov model (HMM) over three time steps. In a Bayes net, each node is
associated with a conditional density: the top Markov chain encodes the
prior :math:`P\left( X_{1} \right)` and transition probabilities
:math:`P\left( X_{2} \middle| X_{1} \right)` and
:math:`P\left( X_{3} \middle| X_{2} \right)`, whereas measurements
:math:`Z_{t}` depend only on the state :math:`X_{t}`, modeled by
conditional densities :math:`P\left( Z_{t} \middle| X_{t} \right)`.
Given known measurements :math:`z_{1}`, :math:`z_{2}` and :math:`z_{3}`
we are interested in the hidden state sequence
:math:`\left( {X_{1},X_{2},X_{3}} \right)` that maximizes the posterior
probability
:math:`P\left( X_{1},X_{2},X_{3} \middle| Z_{1} = z_{1},Z_{2} = z_{2},Z_{3} = z_{3} \right)`.
Since the measurements :math:`Z_{1}`, :math:`Z_{2}`, and :math:`Z_{3}`
are *known*, the posterior is proportional to the product of six
**factors**, three of which derive from the the Markov chain, and three
likelihood factors defined as
:math:`L\left( {X_{t};z} \right) \propto P\left( Z_{t} = z \middle| X_{t} \right)`:

.. math:: P\left( X_{1},X_{2},X_{3} \middle| Z_{1},Z_{2},Z_{3} \right) \propto P\left( X_{1} \right)P\left( X_{2} \middle| X_{1} \right)P\left( X_{3} \middle| X_{2} \right)L\left( {X_{1};z_{1}} \right)L\left( {X_{2};z_{2}} \right)L\left( {X_{3};z_{3}} \right)

|image: 3\_Users\_dellaert\_git\_github\_doc\_images\_hmm-FG.png| Figure
2: An HMM with observed measurements, unrolled over time, represented as
a factor graph.

This motivates a different graphical model, a **factor graph**, in which
we only represent the unknown variables :math:`X_{1}`, :math:`X_{2}`,
and :math:`X_{3}`, connected to factors that encode probabilistic
information on them, as in Figure `2 <#fig_HMM_FG>`__. To do maximum
a-posteriori (MAP) inference, we then maximize the product

.. math:: f\left( {X_{1},X_{2},X_{3}} \right) = \prod f_{i}\left( \mathcal{X}_{i} \right)

\ i.e., the value of the factor graph. It should be clear from the
figure that the connectivity of a factor graph encodes, for each factor
:math:`f_{i}`, which subset of variables :math:`\mathcal{X}_{i}` it
depends on. In the examples below, we use factor graphs to model more
complex MAP inference problems in robotics.
