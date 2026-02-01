import numpy as np
from gtsam import Symbol


def make_key(character, index, cardinality):
    """
    Helper function to mimic the behavior of gtbook.Variables discrete_series function.
    """
    symbol = Symbol(character, index)
    key = symbol.key()
    return (key, cardinality)


def generate_transition_cpt(num_states, transitions=None):
    """
    Generate a row-wise CPT for a transition matrix.
    """
    if transitions is None:
        # Default to identity matrix with slight regularization
        transitions = np.eye(num_states) + 0.1 / num_states

    # Ensure transitions sum to 1 if not already normalized
    transitions /= np.sum(transitions, axis=1, keepdims=True)
    return " ".join(["/".join(map(str, row)) for row in transitions])


def generate_observation_cpt(num_states, num_obs, desired_state):
    """
    Generate a row-wise CPT for observations with contrived probabilities.
    """
    obs = np.zeros((num_states, num_obs + 1))
    obs[:, -1] = 1  # All states default to measurement num_obs
    obs[desired_state, 0:-1] = 1
    obs[desired_state, -1] = 0
    return " ".join(["/".join(map(str, row)) for row in obs])
