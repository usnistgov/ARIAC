.. _SCORING:

========
Scoring
========


There are 3 major components of the Trial Score in ARIAC 2023.

  1. `Cost Factor`: How much does the system (sensors) cost?
  2. `Efficiency Factor`: How fast or efficiently did the system complete the task(s)
  3. `Completion Score`: How well did the task(s) get performed? Are all the correct parts there in the proper place?

Cost Factor
-----------

The Cost Factor :math:`CF` compares the cost of the sensors chosen by the team to the average of all sensor configurations across all teams.

  * :math:`TC` is the total cost of the sensors in the team's configuration.
  * :math:`TC_{avg}` is the average sensor cost across all teams.
  * :math:`w_c` is a weighting constant for cost factor.

  .. math::

    CF = w_c \cdot \frac{TC_{avg}}{TC}


Efficiency Factor
-----------------

The Efficiency Factor :math:`EF_i` for order :math:`i` compares the time to complete order :math:`i` for the team to the average of all teams's times to complete order :math:`i`.

  * :math:`T_i` is the time to complete order :math:`i`
  * :math:`T_{avg_{i}}` is the average time to complete order :math:`i` for all teams
  * :math:`w_t` is a weighting constant for efficiency factor.


  .. math::

    EF_i = w_t \cdot \frac{TC_{avg_{i}}}{T_i}


Completion Score
-----------------

Completion score varies between Kitting, Assembly, and Combined tasks. Each task is generated from Boolean conditions.

Kitting Task Score
^^^^^^^^^^^^^^^^^^^

  * A kitting task has :math:`n` parts that need to be placed on the kitting tray.
  * A shipment has :math:`m` parts on the kitting tray.
  * For each task there are two Boolean conditions:
  
      1. :math:`isCorrectTrayID \rightarrow A` is true if the shipment tray ID matches the kitting tray ID.
      2. :math:`isCorrectDestination` is true if the shipment was sent to the correct destination (as1, as2, as3, as4, kitting, or warehouse).
  * For each quadrant `q` of the kitting tray there are four Boolean conditions:
  
      1. :math:`isCorrectType_{q} \rightarrow B` is true if the part type in quadrant :math:`q` is correct.
      2. :math:`isCorrectColor_{q} \rightarrow C` is true if the part color in quadrant :math:`q` is correct.
      3. :math:`isFlipped_{q} \rightarrow D` is true if the part in quadrant :math:`q` is still flipped.
      4. :math:`isFaulty_{q} \rightarrow E` is true if the part in quadrant :math:`q` is faulty.


.. admonition:: Tray Score

   .. math::

        \texttt{pt}_{tray} = \begin{cases}
        3, &\text{if} ~~ A \\
        0, &\text{otherwise}  \\
        \end{cases}
  
.. admonition:: Quadrant Score

   .. math::

        \texttt{pt}_q = \begin{cases}
        0, &\text{if} ~~ \lnot B \lor E \\
        3, &\text{if} ~~ B \land C \land \lnot D \land \lnot E\\
        2, &\text{if} ~~ B \land \lnot C \land \lnot D \land \lnot E\\
        2, &\text{if} ~~ B \land C \land  D \land \lnot E\\
        1, &\text{if} ~~ B \land \lnot C \land D \land \lnot E\\
        \end{cases}

.. admonition:: Bonus Score

   .. math::

        \texttt{pt}_b = \begin{cases}
        n, &\text{if} ~~ \sum_{q}^{n}{\texttt{pt}_q} = n\times 3 \\
        0, &\text{otherwise} \\
        \end{cases}
   
.. admonition:: Penalty

  A penalty is only applied if more parts are on the tray than needed.

   .. math::

        \texttt{pn}_{ep} = \begin{cases}
        m - n, &\text{if} ~~ m>n \\
        0, &\text{otherwise} \\
        \end{cases}

.. admonition:: Destination Score

   .. math::

        \texttt{destination} = \begin{cases}
        1, &\text{if}\, isCorrectDestination\, \text{is true} \\
        0, &\text{otherwise} \\
        \end{cases}

.. admonition:: Kitting Task Score
  :class: tip
  :name: task-score

   .. math::

        S_{k} = (\max{[\texttt{pt}_{tray} + \sum_{q}^{n}{\texttt{pt}_q} + \texttt{pt}_b - \texttt{pn}_{ep} , 0]}) \times (\texttt{destination})


Assembly Task Score
^^^^^^^^^^^^^^^^^^^

  * An assembly task has :math:`n` parts that need to be assembled into the insert.
  * For each task there is one Boolean condition:

      1. :math:`isCorrectStation` is true if the assembly was done at the correct station (as1, as2, as3, or as4).
  * Each slot `s` in the insert has the following Boolean conditions:

      1. :math:`isAssembled_{s} \rightarrow A` is true if the part in slot :math:`s` is assembled. This implicitly means that the part is of the correct type.
      2. :math:`isCorrectColor_{s} \rightarrow B` is true if the part in slot :math:`s` is of correct color.
      3. :math:`isCorrectPose_{s} \rightarrow C` is true if the part in slot :math:`s` has the correct pose.


.. admonition:: Slot Score

   .. math::

        \texttt{pt}_s = \begin{cases}
        3, &\text{if} ~~ A \land (B \land C)\\
        2, &\text{if} ~~ A \land (B \lor C)\\
        1, &\text{if} ~~ A \land (\lnot B \land \lnot C)\\
        0, &\text{if} ~~ \lnot A \\
        \end{cases}

.. admonition:: Bonus Score

   .. math::

        \texttt{pt}_b = \begin{cases}
        n \times 4, &\text{if} ~~ \sum_{s}^{n}{\texttt{pt}_{s}} = n\times 3 \\
        0, &\text{otherwise} \\
        \end{cases}

.. admonition:: Station Score

   .. math::

        \texttt{station} = \begin{cases}
        1, &\text{if}\, isCorrectStation\, \text{is}\, \text{true} \\
        0, &\text{otherwise} \\
        \end{cases}

.. admonition:: Assembly Task Score
  :class: tip
  :name: task-score-assembly

   .. math::

        S_{a} = (\sum_{s}^{n}{\texttt{pt}_s} + \texttt{pt}_b) \times (\texttt{station})



Combined Task Score
^^^^^^^^^^^^^^^^^^^

  * A combined task has :math:`n` parts that need to be gathered from the environment and assembled to the insert.
  * For each task there is one Boolean condition:

      1. :math:`isCorrectStation` is true if the assembly was done at the correct station (as1, as2, as3, or as4).
  * Each slot `s` in the insert has the following Boolean conditions:
  
      1. :math:`isAssembled_{s} \rightarrow A` is true if the part in slot :math:`s` is assembled. This implicitly means that the part is of the correct type.
      2. :math:`isCorrectColor_{s} \rightarrow B` is true if the part in slot :math:`s` is of correct color.
      3. :math:`isCorrectPose_{s} \rightarrow C` is true if the part in slot :math:`s` has the correct pose.



.. admonition:: Slot Score

   .. math::

        \texttt{pt}_s = \begin{cases}
        0, &\text{if} ~~ \lnot A \\
        5, &\text{if} ~~ A \land (B \land C)\\
        4, &\text{if} ~~ A \land (B \lor C)\\
        3, &\text{if} ~~ A \land (\lnot B \land \lnot C)\\
        \end{cases}

.. admonition:: Bonus Score

   .. math::

        \texttt{pt}_b = \begin{cases}
        n \times 4, &\text{if} ~~ \sum_{s}^{n}{\texttt{pt}_{s}} = n\times 5 \\
        0, &\text{otherwise} \\
        \end{cases}

.. admonition:: Station Score

   .. math::

        \texttt{station} = \begin{cases}
        1, &\text{if}\: isCorrectStation\: \text{is true} \\
        0, &\text{otherwise} \\
        \end{cases}

.. admonition:: Combined Task Score
  :class: tip
  :name: task-score-combined

   .. math::

        S_{c} = (\sum_{s}^{n}{\texttt{pt}_s} + \texttt{pt}_b) \times (\texttt{station})




Completion Score
^^^^^^^^^^^^^^^^^^^
The final completion score :math:`CompletionScore` combines the kitting, assembly, and combined task scores present in that trial.


.. admonition:: Completion Score
  :class: tip
  :name: completion-score

   .. math::

        CompletionScore = \sum_{i=0}^{n_k}{S_{k_i}} + \sum_{j=0}^{n_a}{S_{a_j}} + \sum_{k=0}^{n_c}{S_{c_k}}



Trial Score
-----------------------

The trial score :math:`TrialScore` combines the cost factor, efficiency factors and completion scores into a single score for ranking the teams.


.. admonition:: Trial Score
  :class: caution
  :name: trial-score

   .. math::

        TrialScore = CF \times \sum_{i=0}^{n}{(h_i \times EF_i \times CS_i)}
