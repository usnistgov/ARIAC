========
Scoring
========

There are three major components of the Trial Score in ARIAC 2023.

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

  * A kitting order has :math:`n` parts that need to be placed on the kitting tray.
  * A shipment has :math:`m` parts on the kitting tray.
  * For each task there are two Boolean conditions:
  
      1. :math:`isCorrectTrayID \rightarrow A` is true if the shipment tray ID matches the kitting tray ID.
      2. :math:`isCorrectDestination` is true if the shipment was sent to the correct destination (as1, as2, as3, as4, kitting, or warehouse).
  * For each quadrant `q` of the kitting tray there are four Boolean condition:
  
      1. :math:`isCorrectType_{q} \rightarrow B` is true if the part type in quadrant :math:`q` is correct.
      2. :math:`isCorrectColor_{q} \rightarrow C` is true if the part color in quadrant :math:`q` is correct.
      3. :math:`isFlipped_{q} \rightarrow D` is true if the part in quadrant :math:`q` is still flipped.
      4. :math:`isFaulty_{q} \rightarrow E` is true if the part in quadrant :math:`q` is faulty.


  * Tray score:

    .. math::
      
      \texttt{pt}_{tray} = \begin{cases}
      3, &\text{if} ~~ A \\
      0, &\text{otherwise}  \\
      \end{cases}

  * Quadrant score:

  .. math::

    \texttt{pt}_q = \begin{cases}
    0, &\text{if} ~~ \lnot B \lor E \\
    3, &\text{if} ~~ B \land C \land \lnot D \land \lnot E\\
    2, &\text{if} ~~ B \land \lnot C \land \lnot D \land \lnot E\\
    2, &\text{if} ~~ B \land C \land  D \land \lnot E\\
    1, &\text{if} ~~ B \land \lnot C \land D \land \lnot E\\
    \end{cases}

  * Bonus score:

  .. math::

    \texttt{pt}_b = \begin{cases}
    n, &\text{if} ~~ \sum_{q}^{n}{\texttt{pt}_q} = n\times 3 \\
    0, &\text{otherwise} \\
    \end{cases}

  * Penalty score for having more parts than required:
  .. math::

    \texttt{pn}_{ep} = \begin{cases}
    m - n, &\text{if} ~~ m>n \\
    0, &\text{otherwise} \\
    \end{cases}

  * Destination score:

  .. math::

    \texttt{destination} = \begin{cases}
    1, &\text{if}\, isCorrectDestination=true \\
    0, &\text{otherwise} \\
    \end{cases}

  * Kitting task score:
  
  .. math::

    S_{k} = (\max{[\texttt{pt}_{tray} + \sum_{q}^{n}{\texttt{pt}_q} + \texttt{pt}_b - \texttt{pn}_{eq} , 0]}) \times (\texttt{destination})