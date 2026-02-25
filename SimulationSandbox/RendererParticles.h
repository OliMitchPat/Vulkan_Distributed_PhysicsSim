#include <vulkan/vulkan.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct GpuParticleInstance { glm::vec4 pos_size; glm::vec4 color; };

void createParticleInstanceBuffers();
void destroyParticleInstanceBuffers();
void updateParticleInstanceBuffer(uint32_t currentFrame);

void createParticlePipelineLayout();
void createParticlePipeline();
void destroyParticlePipeline();

void recordParticles(VkCommandBuffer cmd, uint32_t currentFrame);
