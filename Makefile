# Convenience targets.  Everything here is a one-liner you could type yourself.
PY := .venv/bin/python

.PHONY: help venv test lint stand dither suite eval clean urdf preflight

help:
	@echo "make venv       create .venv with uv and install this project"
	@echo "make test       run the test suite (mock backend, ~25 s)"
	@echo "make stand      experiment 01 on the mock backend, evaluated"
	@echo "make dither     experiment 03 -- the central experiment"
	@echo "make suite      every experiment on the mock backend"
	@echo "make eval       evaluate the latest run of each experiment + phase gates"
	@echo "make urdf       regenerate assets/reflex_quad.urdf from config/robot.yaml"
	@echo "make preflight  verify the Isaac API (run inside the Isaac environment)"
	@echo "make clean      remove logs and caches"

venv:
	uv venv --python 3.11 .venv
	uv pip install --python $(PY) -e ".[dev]"

test:
	$(PY) -m pytest

lint:
	uvx ruff check .

stand:
	$(PY) -m reflex_quad 01_stand --backend mock --eval

dither:
	$(PY) -m reflex_quad 03_dither --backend mock --eval

suite:
	@for e in 01_stand 02_uneven_ground 03_dither 03b_dither_all \
	          04_leg_unload 05_fault 06_first_step 07_self_check; do \
		$(PY) -m reflex_quad $$e --backend mock --quiet || exit 1; \
	done

eval:
	$(PY) -m eval.cli --all --phase phase1 --phase phase2 --phase phase3 --phase phase4

urdf:
	$(PY) -m reflex_quad.asset_builder

preflight:
	python scripts/isaac_preflight.py

clean:
	rm -rf logs/2* logs/0* logs/host_* .pytest_cache .ruff_cache
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
