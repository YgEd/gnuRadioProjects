import numpy as np
import pandas as pd
from pgmpy.models import DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination
from tqdm import tqdm
import sys
from pgmpy.estimators import MaximumLikelihoodEstimator

def bin_gain(gain_db):
    """Bin RX gain setting."""
    if gain_db < 20:
        return 'low'
    elif gain_db <= 50:
        return 'nominal'
    else:
        return 'high'

def bin_snr(snr_db):
    """
    Bin SNR. Thresholds placed in flat regions of GFSK erfc curve,
    not on the steep knee (10-18 dB), so bin membership is stable.
    """
    if snr_db < 10:
        return 'poor'
    elif snr_db <= 18:
        return 'marginal'
    else:
        return 'good'

def bin_freq_offset(offset_hz):
    """Bin absolute frequency offset from carrier."""
    offset_hz = abs(offset_hz)
    if offset_hz < 1000:
        return 'low'
    elif offset_hz <= 5000:
        return 'medium'
    else:
        return 'high'

def bin_doppler(doppler_hz):
    """Bin absolute Doppler shift."""
    doppler_hz = abs(doppler_hz)
    if doppler_hz < 100:
        return 'negligible'
    elif doppler_hz <= 500:
        return 'moderate'
    else:
        return 'severe'

def bin_jitter(jitter_ns_rms):
    """Bin RMS timing jitter."""
    if jitter_ns_rms < 10:
        return 'low'
    elif jitter_ns_rms <= 50:
        return 'medium'
    else:
        return 'high'

def bin_channel_noise(noise_dbm):
    """Bin channel noise / interference floor."""
    if noise_dbm < -100:
        return 'low'
    elif noise_dbm <= -85:
        return 'moderate'
    else:
        return 'high'

def bin_ber(ber):
    """Bin bit error rate."""
    if ber < 1e-4:
        return 'excellent'
    elif ber <= 1e-2:
        return 'acceptable'
    else:
        return 'poor'

def discretize_observation(obs: dict) -> dict:
    """
    Convert a dictionary of raw continuous measurements to discrete states.

    Parameters
    ----------
    obs : dict with keys:
        gain_db, snr_db, freq_offset_hz, doppler_hz,
        jitter_ns, channel_noise_dbm, ber, sync_valid (bool)

    Returns
    -------
    dict of discrete states ready for BN inference
    """
    return {
        'Gain':          bin_gain(obs['gain_db']),
        'ChannelNoise':  bin_channel_noise(obs['channel_noise_dbm']),
        'SNR':           bin_snr(obs['snr_db']),
        'FreqOffset':    bin_freq_offset(obs['freq_offset_hz']),
        'Doppler':       bin_doppler(obs['doppler_hz']),
        'Jitter':        bin_jitter(obs['jitter_ns']),
        'SyncWordValid': 'valid' if obs['sync_valid'] else 'invalid',
        'BER':           bin_ber(obs['ber']),
    }


def update_cpd_from_observations(model, observations_df: pd.DataFrame):
    
    

    model.fit(
        observations_df,
        estimator=MaximumLikelihoodEstimator,
        # Equivalent sample size for Bayesian smoothing — prevents zero probabilities
        # from rare combinations that haven't been observed yet
        # he `equivalent_sample_size` parameter adds a form of smoothing called a **Dirichlet prior** or **Laplace smoothing**. 
        # Conceptually it pretends you already observed a small phantom dataset of that size before your real data, spread evenly across all states. 
        # An `equivalent_sample_size=5` with 3 SNR states means it acts as if you added 5/3 ≈ 1.67 phantom observations to each state count before computing probabilities.
#         So for that unobserved parent combination instead of getting 0/0 = undefined, you get roughly:
#             poor:     1.67 / (0 + 5)  = 0.33
#             marginal: 1.67 / (0 + 5)  = 0.33
#             good:     1.67 / (0 + 5)  = 0.33

        equivalent_sample_size=5
    )
    print("CPDs updated from observations.")
    return model


def build_model():

    model = DiscreteBayesianNetwork([
        ('Gain', 'SNR'),
        ('ChannelNoise', 'SNR'),
        ('SNR', 'BER'),
        ('FreqOffset', 'BER'),
        ('Doppler', 'BER'),
        ('Jitter', 'BER'),
        ('BER', 'PacketSuccess'),
        ('SyncWordValid', 'PacketSuccess')
    ])

    # Gain -- assumes nominal is most common operating point
    cpd_gain = TabularCPD(
        variable='Gain',
        variable_card=3,
        values=[[0.10], [0.8], [0.1]],
        state_names={'Gain': ['low', 'nominal', 'high']}
    )

    # Noise -- assumes moderate noise is most common
    cpd_noise = TabularCPD(
        variable='ChannelNoise',
        variable_card=3,
        values=[[0.3], [0.5], [0.2]],
        state_names={'ChannelNoise': ['low', 'moderate', 'high']}
    )

    # Frequency offset - assume is usually low as bladeRF is high quality SDR with a Temperature Compensated Crysatl Oscillator
    cpd_freq = TabularCPD(
        variable='FreqOffset',
        variable_card=3,
        values=[[0.7],[0.2],[0.1]],
        state_names={'FreqOffset': ['low', 'moderate', 'high']}
    )

    # Doppler - assumes usually slow moving or static TX RX pair, should we even use if they'll always be stationary?
    cpd_dopper = TabularCPD(
        variable='Doppler',
        variable_card=3,
        values=[[0.6], [0.3], [0.1]],
        state_names={'Doppler':['negligible', 'moderate', 'severe']}
    )


    # Jitter - assumes BladeRF timing is generally good
    cpd_jitter = TabularCPD(
        variable='Jitter',
        variable_card=3,
        values=[[0.7], [0.25], [0.05]],
        state_names={'Jitter': ['low', 'medium','high']}
    )

    # Sync word - assuming well-designed sync word
    cpd_sync = TabularCPD(
        variable='SyncWordValid',
        variable_card=2,
        values=[[0.9], [0.1]],
        state_names={'SyncWordValid': ['valid', 'invalid']}
    )


    # SNR CPD P(SNR | Gain, Channel Noise)
    # Gain: [low, nominal, high] * ChannelNoise: [low, moderate, high]
    #        0.1  0.8      0.1                    0.3    0.5     0.2
    # Column order: all combination of parent states

    cpd_snr = TabularCPD(
        variable='SNR',
        variable_card=3,
        # poor SNR probability
        values=[
            # These values in this matrix are all from "expert domain knowledge that will be updated"
            # col 1,2,3 Gain = low
            # col 4,5,6 Gain = nominal
            # col 7,8,9 Gain = high
            # col 1,4,7 ChannelNoise = low
            # col 2,5,8 ChannelNoise = moderate
            # col 3,6,9 ChannelNoise = high
            # row 1 SNR = poor
            # row 2 SNR = marginal
            # row 3 SNR = good
            # """
            # [p(SNR=poor | Gain=low & CN=low), p(SNR=poor | Gain=low & CN=moderate), p(SNR=poor | Gain=low & CN=high), 
            #     p(SNR=poor | Gain=nominal & CN=low), p(SNR=poor | Gain=nominal & CN=moderate), p(SNR=poor | Gain=nominal & CN=high),
            #         p(SNR=poor | Gain=high & CN=low), p(SNR=poor | Gain=high & CN=moderate), p(SNR=poor | Gain=high & CN=high)],
            # [p(SNR=marginal | Gain=low & CN=low)], p(SNR=marginal | Gain=low & CN=moderate), p(SNR=marginal | Gain=low & CN=high), 
            #     p(SNR=marginal | Gain=nominal & CN=low), p(SNR=marginal | Gain=nominal & CN=moderate), p(SNR=marginal | Gain=nominal & CN=high),
            #         p(SNR=marginal | Gain=high & CN=low), p(SNR=marginal | Gain=high & CN=moderate), p(SNR=marginal | Gain=high & CN=high)],
            # [p(SNR=good | Gain=low & CN=low), p(SNR=good | Gain=low & CN=moderate), p(SNR=good | Gain=low & CN=high), 
            #     p(SNR=good | Gain=nominal & CN=low), p(SNR=good | Gain=nominal & CN=moderate), p(SNR=good | Gain=nominal & CN=high),
            #         p(SNR=good | Gain=high & CN=low), p(SNR=good | Gain=high & CN=moderate), p(SNR=good | Gain=high & CN=high)]]

            # """
            [0.70, 0.80, 0.90, 
                0.20, 0.30, 0.50,
                    0.10, 0.15, 0.45], 
            [0.20, 0.15, 0.08, 
                0.40, 0.45, 0.35, 
                    0.30, 0.35, 0.40], 
            [0.10, 0.05, 0.02, 
                0.40, 0.25, 0.15, 
                    0.60, 0.50, 0.15],
        ],
        evidence=['Gain', 'ChannelNoise'],
        evidence_card=[3, 3],
        state_names={
            'SNR': ['poor', 'marginal', 'good'],
            'Gain': ['low', 'nominal', 'high'],
            'ChannelNoise':['low', 'moderate', 'high']
        }
        
    )


    # BER CPD P(BER | SNR, FreqOffset, Doppler, Jitter)
    # 3 x 3 x 3 x 3 = 81 parent state combinations

    snr_states = ['poor', 'marginal', 'good']
    freq_states = ['low', 'moderate', 'high']
    doppler_states = ['negligible', 'moderate', 'severe']
    jitter_states = ['low', 'medium', 'high']

    snr_score     = {'poor': 3, 'marginal': 1, 'good': 0}
    freq_score    = {'low': 0, 'moderate': 1, 'high': 2}
    doppler_score = {'negligible': 0, 'moderate': 1, 'severe': 2}
    jitter_score  = {'low': 0, 'medium': 1, 'high': 2}

    ber_columns = []

    for snr in snr_states:
        for freq in freq_states:
            for doppler in doppler_states:
                for jitter in jitter_states:
                    score = (
                        snr_score[snr] + freq_score[freq] + doppler_score[doppler] + jitter_score[jitter]
                    )

                    if score <= 1:
                        col = [0.90, 0.09, 0.01] # BER Good, acceptable, poor
                    if score <=3:
                        col = [0.50, 0.50, 0.10]
                    if score <=5:
                        col = [0.15, 0.55, 0.30]
                    if score <=7:
                        col = [0.05, 0.35, 0.60]
                    else:
                        col = [0.01, 0.09, 0.90]
                    
                    ber_columns.append(col)

    # pgmpy expects shhape (n_states, n_parent_combos) by default right now our "col" variables are actually rows
    # Have to transpose

    ber_values = np.array(ber_columns).T

    cpd_ber = TabularCPD(
        variable='BER',
        variable_card = 3,
        values=ber_values,
        evidence=['SNR', 'FreqOffset', 'Doppler', 'Jitter'],
        evidence_card=[3, 3, 3, 3],
        state_names={
            'BER': ['good', 'acceptable', 'poor'],
            'SNR': snr_states,
            'FreqOffset': freq_states,
            'Doppler': doppler_states,
            'Jitter': jitter_states
        }
    )


    # Packet success = P(PacketSuccess | BER, SyncWordValid)

    cpd_packet = TabularCPD(
        variable='PacketSuccess',
        variable_card = 2,

        # Shape of values is determined by the possible outcomes of your current child state in this case 'PacketSuccess' x (combination of parent states)
        # in this case PacketSuccess can be 'success' or 'failure'
        # Two parent states are BER that has 3 outcomes and SyncWordValid that has 2 outcomes 
        # Values shape is then 2x3*2 == 2x6
        values=[
        
            # col 1,4 BER=good
            # col 2,5 BER=acceptable
            # col 3,6 BER=poor
            # col 1,2,3 SyncWV=Valid
            # col 4,5,6 SyncWV=Invalid
            [0.99, 0.75, 0.15, 0.10, 0.05, 0.02], #PacketSuccess=success
            [0.01, 0.25, 0.85, 0.90, 0.95, 0.98] #PacketSuccess=failure
        ],
        evidence=['BER', 'SyncWordValid'],
        evidence_card=[3, 2],
        state_names={
            'PacketSuccess': ['success', 'failure'],
            'BER': ['good', 'acceptable', 'poor'],
            'SyncWordValid': ['valid', 'invalid']
        }

    )

    model.add_cpds(
        cpd_gain, cpd_noise, cpd_freq, cpd_dopper, cpd_jitter, cpd_sync, cpd_snr, cpd_ber, cpd_packet
    )

    try:
        assert model.check_model()
    except Exception as e:
        print("Model CPDs are inconsistent - check probability columns sum up to 1")
        print(f"Problem with BNM: {e}")
        sys.exit(1)
        
    
    return model

def query_packet_success(model, evidence: dict):
   
    infer = VariableElimination(model)
    result = infer.query(variables=['PacketSuccess'], evidence=evidence)
    states = result.state_names['PacketSuccess']
    values = result.values
    return dict(zip(states, values))

if __name__ == "__main__":
    model = build_model()
    print("model succesfully built!")
     # --- Forward inference: predict packet success from observed conditions ---
    evidence = {
        'SNR':          'marginal',
        'FreqOffset':   'low',
        'Doppler':      'negligible',
        'Jitter':       'medium',
        'SyncWordValid':'valid',
    }
    result = query_packet_success(model, evidence)
    print("P(PacketSuccess | conditions):")
    for state, prob in result.items():
        print(f"  {state}: {prob:.3f}")