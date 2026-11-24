"""One training round: generate clips, augment, train, export, evaluate.

  run_train.py configs/hey_rainbow.yml [--steps generate,augment,train,eval] [--tag NAME]

Steps already done are skipped by train.py itself (clips and features are reused), so a
retry with only training changes (steps, layer size, negative weight, batch mix) starts at
training: delete /data/runs/<model>/*features*.npy to redo augmentation, or the clip
directories to regenerate. The config is copied next to the results of each round in
/data/results/<model>/<tag>/ with the eval numbers and the .onnx.
"""

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import yaml

TRAIN = '/opt/openWakeWord/openwakeword/train.py'
CUSTOM_NEGATIVE_SHARE = 0.4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('config')
    ap.add_argument('--steps', default='generate,augment,train,eval')
    ap.add_argument('--tag', default=time.strftime('%Y%m%d-%H%M%S'))
    a = ap.parse_args()
    steps = a.steps.split(',')

    config = yaml.safe_load(open(a.config))
    name = config['model_name']
    run_dir = Path(config['output_dir']) / name
    run_dir.mkdir(parents=True, exist_ok=True)

    # train.py picks negative texts in turn from custom + ~n_samples generated phrases, so a
    # handful of custom phrases would almost never be spoken: repeat them to ~40% of the list.
    custom = config.get('custom_negative_phrases', [])
    if custom:
        k = max(1, round(CUSTOM_NEGATIVE_SHARE * config['n_samples'] / len(custom)))
        config['custom_negative_phrases'] = custom * k
    runtime = run_dir / 'config.runtime.yml'
    yaml.safe_dump(config, open(runtime, 'w'))

    flags = {'generate': '--generate_clips', 'augment': '--augment_clips', 'train': '--train_model'}
    for step in ('generate', 'augment', 'train'):
        if step in steps:
            print(f'##### {name}: {step}', flush=True)
            subprocess.run([sys.executable, TRAIN, '--training_config', str(runtime), flags[step]],
                           check=True)

    out = Path('/data/results') / name / a.tag
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(a.config, out / 'config.yml')
    model = Path(config['output_dir']) / f'{name}.onnx'
    if 'train' in steps:
        shutil.copy(model, out / f'{name}.onnx')
    if 'eval' in steps:
        print(f'##### {name}: eval', flush=True)
        subprocess.run([sys.executable, '/src/eval.py', '--model', f'{name}={out / (name + ".onnx")}',
                        '--positives', str(run_dir / 'positive_test'),
                        '--negatives', str(run_dir / 'negative_test'),
                        '--json', str(out / 'eval.json')], check=True)
        word = run_dir / 'positive_test_word'   # the name alone (add_word.py)
        if word.is_dir():
            print(f'##### {name}: eval of the name alone', flush=True)
            subprocess.run([sys.executable, '/src/eval.py', '--model',
                            f'{name}_word={out / (name + ".onnx")}',
                            '--positives', str(word), '--negatives', str(run_dir / 'negative_test'),
                            '--json', str(out / 'eval_word.json')], check=True)
    print(f'##### {name}: results in {out}', flush=True)


if __name__ == '__main__':
    main()
