AIDO_REGISTRY ?= docker.io
PIP_INDEX_URL ?= https://pypi.org/simple

repo=aidonode-random_agent-full
# repo=$(shell basename -s .git `git config --get remote.origin.url`)
branch=$(shell git rev-parse --abbrev-ref HEAD)
tag=$(AIDO_REGISTRY)/duckietown/$(repo):$(branch)

update-reqs:
	pur --index-url $(PIP_INDEX_URL) -r requirements.txt -f -m '*' -o requirements.resolved
	aido-update-reqs requirements.resolved

build: update-reqs
	docker build --pull \
			--build-arg  AIDO_REGISTRY=$(AIDO_REGISTRY) \
			-t $(tag) .

build-no-cache: update-reqs
	docker build --pull -t $(tag)  --no-cache .

push: build
	docker push $(tag)

submit: update-reqs
	dts challenges submit


submit-bea: update-reqs
	dts challenges submit --impersonate 1639 --challenge all --retire-same-label

# submit-robotarium:
# 	dts challenges submit --challenge aido2_LF_r_pri,aido2_LF_r_pub

# submit-baseline-sim:
# 	dts challenges submit --impersonate 1639 --user-label "straight" --challenge aido3-LF-sim-validation,aido3-LF-sim-testing,aido3-LFV-sim-validation,aido3-LFV-sim-testing,aido3-LFVI-sim-validation,aido3-LFVI-sim-testing
#
# submit-baseline-real-validation:
# 	dts challenges submit --impersonate 1639 --user-label "straight" --challenge aido3-LF-real-validation,aido3-LFV-real-validation

submit-baseline:
	dts challenges submit --impersonate 1639 --user-label "straight" --challenge aido3-LF-sim-validation,aido3-LF-sim-testing,aido3-LFV-sim-validation,aido3-LFV-sim-testing,aido3-LFVI-sim-validation,aido3-LFVI-sim-testing,aido3-LF-real-validation,aido3-LFV-real-validation
