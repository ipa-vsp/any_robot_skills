ARG BASE_IMAGE
FROM ${BASE_IMAGE} as base
FROM ghcr.io/ipa-vsp/docker_envs:main as builder

FROM base as pre_build
RUN mkdir -p /ros_ws/src
COPY . /ros_ws/src
COPY --from=builder workspace.bash /builder/workspace.bash
RUN chmod +x /builder/workspace.bash
ARG ROSINSTALL_CI_JOB_TOKEN=
ENV ROSINSTALL_CI_JOB_TOKEN $ROSINSTALL_CI_JOB_TOKEN
ARG CI_JOB_TOKEN=
ENV CI_JOB_TOKEN $CI_JOB_TOKEN
RUN apt-get update -qq && apt-get install -q -y --no-install-recommends \
    apt-utils \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update -qq
RUN rm -rf /var/lib/apt/lists/*


FROM pre_build as build
ARG CMAKE_ARGS=
ENV CMAKE_ARGS $CMAKE_ARGS
RUN apt-get update -qq && \
    /builder/workspace.bash build_workspace /ros_ws && \
    rm -rf /var/lib/apt/lists/*

FROM build as test
RUN apt-get update -qq && \
    /builder/workspace.bash test_workspace /ros_ws && \
    rm -rf /var/lib/apt/lists/*

FROM test as install
RUN apt-get update -qq && \
    /builder/workspace.bash install_workspace /ros_ws && \
    rm -rf /var/lib/apt/lists/*

FROM base as deploy
COPY --from=install /ros_ws/DEPENDS /ros_ws/DEPENDS
COPY --from=install /ros_entrypoint.sh /ros_entrypoint.sh
COPY --from=builder workspace.bash /builder/workspace.bash
RUN apt-get update -qq && \
    /builder/workspace.bash install_depends /ros_ws && \
    rm -rf /var/lib/apt/lists/*
COPY --from=install /ros_ws/install /ros_ws/install
