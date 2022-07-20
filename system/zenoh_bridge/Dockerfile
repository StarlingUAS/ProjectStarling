ARG VERSION=latest
ARG REGISTRY
FROM eclipse/zenoh-bridge-dds

COPY run.sh /
COPY zenoh.json5 /zenoh_config.json5

ENTRYPOINT []
CMD ["./run.sh"]