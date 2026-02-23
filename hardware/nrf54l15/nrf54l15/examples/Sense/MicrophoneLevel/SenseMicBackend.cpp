#include <Arduino.h>
#include <string.h>

#include "SenseMicBackend.h"

#if defined(__has_include)
#if __has_include(<generated/zephyr/autoconf.h>)
#include <generated/zephyr/autoconf.h>
#endif
#if __has_include(<zephyr/device.h>) && __has_include(<zephyr/devicetree.h>) && __has_include(<zephyr/kernel.h>) && __has_include(<zephyr/audio/dmic.h>)
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>
#define XIAO_SENSE_HAS_DMIC_API 1
#else
#define XIAO_SENSE_HAS_DMIC_API 0
#endif
#else
#define XIAO_SENSE_HAS_DMIC_API 0
#endif

#if XIAO_SENSE_HAS_DMIC_API
#if DT_NODE_EXISTS(DT_ALIAS(dmic20))
#define XIAO_SENSE_DMIC_NODE_PRESENT 1
static const struct device *const g_dmic = DEVICE_DT_GET(DT_ALIAS(dmic20));
#else
#define XIAO_SENSE_DMIC_NODE_PRESENT 0
#endif
#else
#define XIAO_SENSE_DMIC_NODE_PRESENT 0
#endif

#if XIAO_SENSE_HAS_DMIC_API && XIAO_SENSE_DMIC_NODE_PRESENT
#define XIAO_SENSE_DMIC_SAMPLE_RATE_HZ 16000U
#define XIAO_SENSE_DMIC_BIT_WIDTH 16U
#define XIAO_SENSE_DMIC_BLOCK_SIZE_BYTES 1600U
#define XIAO_SENSE_DMIC_BLOCK_COUNT 4U
#define XIAO_SENSE_DMIC_READ_TIMEOUT_MS 500

K_MEM_SLAB_DEFINE_STATIC(g_dmicMemSlab, XIAO_SENSE_DMIC_BLOCK_SIZE_BYTES, XIAO_SENSE_DMIC_BLOCK_COUNT, 4);
#endif

namespace {
bool g_streaming = false;
bool g_missingMessagePrinted = false;

void printMissingSupportOnce(const char *message)
{
  if (g_missingMessagePrinted) {
    return;
  }
  g_missingMessagePrinted = true;
  Serial.println(message);
}
}  // namespace

void senseMicSetup()
{
#if !XIAO_SENSE_HAS_DMIC_API
  printMissingSupportOnce("DMIC support unavailable: missing Zephyr DMIC headers.");
#elif !XIAO_SENSE_DMIC_NODE_PRESENT
  printMissingSupportOnce("DMIC support unavailable: devicetree alias dmic20 is missing.");
#else
  if (!device_is_ready(g_dmic)) {
    printMissingSupportOnce("DMIC device is not ready.");
    return;
  }

  static struct pcm_stream_cfg streamCfg;
  streamCfg.pcm_rate = XIAO_SENSE_DMIC_SAMPLE_RATE_HZ;
  streamCfg.pcm_width = XIAO_SENSE_DMIC_BIT_WIDTH;
  streamCfg.block_size = XIAO_SENSE_DMIC_BLOCK_SIZE_BYTES;
  streamCfg.mem_slab = &g_dmicMemSlab;

  static struct dmic_cfg dmicCfg;
  memset(&dmicCfg, 0, sizeof(dmicCfg));
  dmicCfg.io.min_pdm_clk_freq = 1000000U;
  dmicCfg.io.max_pdm_clk_freq = 3500000U;
  dmicCfg.io.min_pdm_clk_dc = 40U;
  dmicCfg.io.max_pdm_clk_dc = 60U;
  dmicCfg.streams = &streamCfg;
  dmicCfg.channel.req_num_streams = 1U;
  dmicCfg.channel.req_num_chan = 1U;
  dmicCfg.channel.req_chan_map_lo = dmic_build_channel_map(0U, 0U, PDM_CHAN_LEFT);

  int ret = dmic_configure(g_dmic, &dmicCfg);
  if (ret < 0) {
    Serial.print("dmic_configure failed: ");
    Serial.println(ret);
    return;
  }

  ret = dmic_trigger(g_dmic, DMIC_TRIGGER_START);
  if (ret < 0) {
    Serial.print("DMIC start failed: ");
    Serial.println(ret);
    return;
  }

  g_streaming = true;
  Serial.println("DMIC started. Streaming average + peak amplitude.");
#endif
}

void senseMicLoop()
{
#if XIAO_SENSE_HAS_DMIC_API && XIAO_SENSE_DMIC_NODE_PRESENT
  if (!g_streaming) {
    delay(500);
    return;
  }

  void *buffer = nullptr;
  size_t size = 0U;
  const int ret = dmic_read(g_dmic, 0, &buffer, &size, XIAO_SENSE_DMIC_READ_TIMEOUT_MS);
  if (ret < 0) {
    Serial.print("dmic_read failed: ");
    Serial.println(ret);
    delay(200);
    return;
  }

  const int16_t *samples = static_cast<const int16_t *>(buffer);
  const size_t sampleCount = size / sizeof(int16_t);
  if (sampleCount == 0U) {
    k_mem_slab_free(&g_dmicMemSlab, buffer);
    delay(100);
    return;
  }

  uint32_t sumAbs = 0U;
  uint16_t peakAbs = 0U;
  for (size_t i = 0; i < sampleCount; ++i) {
    const int32_t value = static_cast<int32_t>(samples[i]);
    const uint16_t magnitude = static_cast<uint16_t>(value < 0 ? -value : value);
    sumAbs += magnitude;
    if (magnitude > peakAbs) {
      peakAbs = magnitude;
    }
  }

  const uint32_t avgAbs = sumAbs / static_cast<uint32_t>(sampleCount);
  Serial.print("samples=");
  Serial.print(sampleCount);
  Serial.print(" avg_abs=");
  Serial.print(avgAbs);
  Serial.print(" peak_abs=");
  Serial.println(peakAbs);

  k_mem_slab_free(&g_dmicMemSlab, buffer);
#else
  printMissingSupportOnce("Sense microphone support is unavailable for this build.");
  delay(1000);
#endif
}
