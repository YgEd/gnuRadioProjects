import numpy as np
import pandas as pd
from pgmpy.models import DiscreteBayesianNetwork
from pgmpy.factors.discrete import TabularCPD
from pgmpy.inference import VariableElimination
from tqdm import tqdm
import sys
from pgmpy.estimators import MaximumLikelihoodEstimator, BayesianEstimator
import os
import time
from graphviz import Digraph
import matplotlib.pyplot as plt
import networkx as nx

class Colors:
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    ENDC = '\033[0m'


# ── Consistent state names ─────────────────────────────────────────

STATES_3 = ['low', 'moderate', 'high']
STATES_PACKET = ['success', 'failure']

# Remap old state names to new consistent ones
STATE_REMAP = {
    'poor': 'low', 'marginal': 'moderate', 'good': 'high',
    'negligible': 'low', 'medium': 'moderate', 'severe': 'high',
    'excellent': 'low', 'acceptable': 'moderate',
    'nominal': 'moderate',
}


# ── Binning functions ──────────────────────────────────────────────

def bin_gain(gain_db):
    """Bin RX gain setting."""
    if gain_db < 20:
        return 'low'
    elif gain_db <= 50:
        return 'moderate'
    else:
        return 'high'

def bin_snr(snr_db):
    """Bin SNR. Thresholds in flat regions of GFSK erfc curve."""
    if snr_db < 10:
        return 'low'
    elif snr_db <= 18:
        return 'moderate'
    else:
        return 'high'

def bin_freq_offset(offset_hz):
    """Bin absolute frequency offset from carrier."""
    offset_hz = abs(offset_hz)
    if offset_hz < 1000:
        return 'low'
    elif offset_hz <= 5000:
        return 'moderate'
    else:
        return 'high'

def bin_doppler(doppler_hz):
    """Bin absolute Doppler shift."""
    doppler_hz = abs(doppler_hz)
    if doppler_hz < 100:
        return 'low'
    elif doppler_hz <= 500:
        return 'moderate'
    else:
        return 'high'

def bin_jitter(jitter_ns_rms):
    """Bin RMS timing jitter."""
    if jitter_ns_rms < 10:
        return 'low'
    elif jitter_ns_rms <= 50:
        return 'moderate'
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
        return 'low'
    elif ber <= 1e-2:
        return 'moderate'
    else:
        return 'high'


def discretize_observation(obs: dict) -> dict:
    """Convert raw continuous measurements to discrete states."""
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


# ── .dne export ────────────────────────────────────────────────────

def export_to_dne(model, saved_state_names, filename="model.dne", network_name="BayesNet", epsilon=1e-6):
    """Export a pgmpy DiscreteBayesianNetwork to Netica .dne format."""
    lines = []
    lines.append('// ~->[DNET-1]->~')
    lines.append(f'bnet {network_name} {{')
    lines.append('    autoupdate = TRUE;')
    lines.append('')

    for cpd in model.cpds:
        var = cpd.variable
        states = saved_state_names[var]
        parents = cpd.variables[1:]
        n_states = len(states)

        safe_var = var.replace(' ', '_')
        safe_states = [s.replace(' ', '_') for s in states]
        safe_parents = [p.replace(' ', '_') for p in parents]

        # Expected number of parent combos
        expected_cols = 1
        for p in parents:
            expected_cols *= len(saved_state_names[p])

        vals = cpd.get_values().copy()

        # Reorder rows if post-fit child states are recognizable but reordered
        post_fit_child = [str(s) for s in cpd.state_names[var]]
        saved_child = [str(s) for s in states]
        if set(post_fit_child) == set(saved_child) and post_fit_child != saved_child:
            row_order = [post_fit_child.index(s) for s in saved_child]
            vals = vals[row_order, :]

        # Reorder columns if post-fit parent states are recognizable but reordered
        if len(parents) > 0:
            parent_cards_actual = [len(cpd.state_names[p]) for p in parents]
            if np.prod(parent_cards_actual) == vals.shape[1]:
                vals_nd = vals.reshape([vals.shape[0]] + parent_cards_actual)

                for pi, p in enumerate(parents):
                    post_fit_parent = [str(s) for s in cpd.state_names[p]]
                    saved_parent = [str(s) for s in saved_state_names[p]]
                    if set(post_fit_parent) == set(saved_parent) and post_fit_parent != saved_parent:
                        perm = [post_fit_parent.index(s) for s in saved_parent]
                        vals_nd = np.take(vals_nd, perm, axis=pi + 1)

                vals = vals_nd.reshape(vals.shape[0], -1)

        # Pad rows if fit() collapsed missing child states
        if vals.shape[0] < n_states:
            padded = np.full((n_states, vals.shape[1]), epsilon)
            padded[:vals.shape[0], :] = vals
            vals = padded

        # Pad columns if fit() dropped parent combos
        if vals.shape[1] < expected_cols:
            padded = np.full((n_states, expected_cols), epsilon)
            padded[:, :vals.shape[1]] = vals
            vals = padded

        # Clean and renormalize
        vals = np.nan_to_num(vals, nan=epsilon, posinf=epsilon, neginf=epsilon)
        vals = np.maximum(vals, epsilon)
        vals = vals / vals.sum(axis=0, keepdims=True)

        # Build node entry
        lines.append(f'    node {safe_var} {{')
        lines.append(f'        kind = NATURE;')
        lines.append(f'        discrete = TRUE;')
        lines.append(f'        states = ({", ".join(safe_states)});')
        lines.append(f'        parents = ({", ".join(safe_parents)});')

        # Flat probs: all child states for combo 0, then combo 1, etc.
        flat_probs = []
        for j in range(vals.shape[1]):
            for i in range(vals.shape[0]):
                flat_probs.append(f"{vals[i, j]:.6f}")
        prob_str = ", ".join(flat_probs)
        lines.append(f'        probs = ({prob_str});')

        lines.append(f'    }};')
        lines.append('')

    lines.append('};')

    with open(filename, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Exported to {filename}")


# ── Model update ───────────────────────────────────────────────────

def update_cpd_from_observations(model, observations_df, saved_state_names):
    """Fit CPDs from observation data using Bayesian estimation."""
    model.fit(
        observations_df,
        estimator=BayesianEstimator,
        equivalent_sample_size=5,
        state_names=saved_state_names
    )
    print("CPDs updated from observations.")
    return model


# ── Model construction ─────────────────────────────────────────────

def build_model():

    model = DiscreteBayesianNetwork([
        ('Gain', 'SNR'),
        ('ChannelNoise', 'SNR'),
        ('SNR', 'BER'),
        ('FreqOffset', 'BER'),
        ('Doppler', 'BER'),
        ('Jitter', 'BER'),
        ('BER', 'PacketSuccess')
    ])

    cpd_gain = TabularCPD(
        variable='Gain', variable_card=3,
        values=[[0.10], [0.80], [0.10]],
        state_names={'Gain': STATES_3}
    )

    cpd_noise = TabularCPD(
        variable='ChannelNoise', variable_card=3,
        values=[[0.30], [0.50], [0.20]],
        state_names={'ChannelNoise': STATES_3}
    )

    cpd_freq = TabularCPD(
        variable='FreqOffset', variable_card=3,
        values=[[0.70], [0.20], [0.10]],
        state_names={'FreqOffset': STATES_3}
    )

    cpd_doppler = TabularCPD(
        variable='Doppler', variable_card=3,
        values=[[0.60], [0.30], [0.10]],
        state_names={'Doppler': STATES_3}
    )

    cpd_jitter = TabularCPD(
        variable='Jitter', variable_card=3,
        values=[[0.70], [0.25], [0.05]],
        state_names={'Jitter': STATES_3}
    )

    # SNR CPD P(SNR | Gain, ChannelNoise)
    cpd_snr = TabularCPD(
        variable='SNR', variable_card=3,
        values=[
            [0.70, 0.80, 0.90,  0.20, 0.30, 0.50,  0.10, 0.15, 0.45],
            [0.20, 0.15, 0.08,  0.40, 0.45, 0.35,  0.30, 0.35, 0.40],
            [0.10, 0.05, 0.02,  0.40, 0.25, 0.15,  0.60, 0.50, 0.15],
        ],
        evidence=['Gain', 'ChannelNoise'], evidence_card=[3, 3],
        state_names={'SNR': STATES_3, 'Gain': STATES_3, 'ChannelNoise': STATES_3}
    )

    # BER CPD P(BER | SNR, FreqOffset, Doppler, Jitter) — 81 parent combos
    snr_score     = {'low': 3, 'moderate': 1, 'high': 0}
    freq_score    = {'low': 0, 'moderate': 1, 'high': 2}
    doppler_score = {'low': 0, 'moderate': 1, 'high': 2}
    jitter_score  = {'low': 0, 'moderate': 1, 'high': 2}

    ber_columns = []
    for snr in STATES_3:
        for freq in STATES_3:
            for doppler in STATES_3:
                for jitter in STATES_3:
                    score = snr_score[snr] + freq_score[freq] + doppler_score[doppler] + jitter_score[jitter]
                    if score <= 1:
                        col = [0.90, 0.09, 0.01]
                    elif score <= 3:
                        col = [0.50, 0.40, 0.10]
                    elif score <= 5:
                        col = [0.15, 0.55, 0.30]
                    elif score <= 7:
                        col = [0.05, 0.35, 0.60]
                    else:
                        col = [0.01, 0.09, 0.90]
                    ber_columns.append(col)

    ber_values = np.array(ber_columns).T

    cpd_ber = TabularCPD(
        variable='BER', variable_card=3,
        values=ber_values,
        evidence=['SNR', 'FreqOffset', 'Doppler', 'Jitter'], evidence_card=[3, 3, 3, 3],
        state_names={'BER': STATES_3, 'SNR': STATES_3, 'FreqOffset': STATES_3,
                     'Doppler': STATES_3, 'Jitter': STATES_3}
    )

    cpd_packet = TabularCPD(
        variable='PacketSuccess', variable_card=2,
        values=[
            [0.95, 0.60, 0.05],
            [0.05, 0.40, 0.95],
        ],
        evidence=['BER'], evidence_card=[3],
        state_names={'PacketSuccess': STATES_PACKET, 'BER': STATES_3}
    )

    model.add_cpds(cpd_gain, cpd_noise, cpd_freq, cpd_doppler, cpd_jitter, cpd_snr, cpd_ber, cpd_packet)

    try:
        assert model.check_model()
    except Exception as e:
        print("Model CPDs are inconsistent - check probability columns sum to 1")
        print(f"Problem with BNM: {e}")
        sys.exit(1)

    return model


def get_state_names(model):
    """Save state names from model before fit() overwrites them."""
    return {cpd.variable: list(cpd.state_names[cpd.variable]) for cpd in model.cpds}


def query_packet_success(model, evidence: dict):
    infer = VariableElimination(model)
    result = infer.query(variables=['PacketSuccess'], evidence=evidence)
    states = result.state_names['PacketSuccess']
    values = result.values
    return dict(zip(states, values))


if __name__ == "__main__":
    model = build_model()
    print("Model successfully built!")

    # Save state names before fit() can overwrite them
    saved_state_names = get_state_names(model)

    file_name = None
    if len(sys.argv) > 1:
        file_name = sys.argv[1]
        if not os.path.isfile(file_name):
            print(f"Cannot reach input file: {file_name}\n Usage: python3 {sys.argv[0]} <optional input file to update cpds>")
            sys.exit(1)

    if file_name is not None:
        observations = []
        rfdf = pd.read_csv(file_name)

        for _, data in rfdf.iterrows():
            observations.append({
                'Gain': bin_gain(data['gain']),
                'SNR': data['snr_bin'],
                'ChannelNoise': data['channelnoise_bin'],
                'FreqOffset': data['freq_offset_bin'],
                'Doppler': data['doppler_bin'],
                'Jitter': data['jitter_bin'],
                'BER': data['ber_bin'],
                'PacketSuccess': data['packet_success'],
            })

        obdf = pd.DataFrame(observations)

        # Remap old state names to new consistent ones
        for col in obdf.columns:
            obdf[col] = obdf[col].map(lambda x: STATE_REMAP.get(x, x))
        
        # ── Data diagnostics ───────────────────────────────────────────
        diag_cols = ['Gain', 'SNR', 'ChannelNoise', 'FreqOffset', 'Doppler', 'Jitter', 'BER', 'PacketSuccess']
        combo_counts = obdf.groupby(diag_cols).size().reset_index(name='count')
        combo_counts = combo_counts.sort_values('count', ascending=False)

        with open('data_diagnostics.txt', 'w') as f:
            f.write(f"Total observations: {len(obdf)}\n")
            f.write(f"Unique conditions: {len(combo_counts)}\n\n")
            
            f.write("=== Per-variable distributions ===\n")
            for col in diag_cols:
                f.write(f"\n{col}:\n")
                vc = obdf[col].value_counts()
                for val, cnt in vc.items():
                    f.write(f"  {val}: {cnt} ({cnt/len(obdf)*100:.1f}%)\n")
            
            f.write(f"\n=== All observed conditions ({len(combo_counts)} unique) ===\n\n")
            f.write(combo_counts.to_string(index=False))
            
            f.write(f"\n\n=== PacketSuccess by BER (crosstab) ===\n\n")
            ct = pd.crosstab(obdf['BER'], obdf['PacketSuccess'])
            f.write(ct.to_string())

        print(f"Total observations: {len(obdf)}")
        print(f"Unique conditions: {len(combo_counts)}")
        print(f"Diagnostics written to data_diagnostics.txt")

        print(f"Updating Model from {file_name}")
        model = update_cpd_from_observations(model, obdf, saved_state_names)
        print("Updated Model!")

        for cpd in model.cpds:
            print(f"=== {cpd.variable} ===")
            print(f"  parents: {cpd.variables[1:]}")
            print(f"  values:\n{cpd.values}")
            print()

    # Export to .dne (works before or after fit)
    export_to_dne(model, saved_state_names, 'rf_bnm.dne', 'rf_bnm')

    print(f"\nBased on the following Keys and Possible Values...\n")
    time.sleep(1)
    for var, states in saved_state_names.items():
        print(f"=== {var} ===")
        print(f"  states: {states}")
    time.sleep(1)

    for _ in range(1000):
        raw_in = input(f"\nEnter key value pair evidence (e.g. SNR high BER low) or {Colors.RED}q to quit{Colors.ENDC}:\n")
        if raw_in.lower().strip() != 'q':
            items = raw_in.split()
            item_kv_pairs = iter(items)
            evidence = dict(zip(item_kv_pairs, item_kv_pairs))

            result = query_packet_success(model, evidence)
            print(f"\nBased on this evidence:\n\n{Colors.YELLOW}{evidence}{Colors.ENDC}\n")
            print(f"{Colors.GREEN}P(PacketSuccess | conditions):")
            for state, prob in result.items():
                print(f"  {state}: {prob:.3f}")
            print(f"{Colors.ENDC}")
        else:
            break