/** @type {import('next').NextConfig} */
const nextConfig = {
  webpack: (config, { isServer }) => {
    // 仅在服务器端构建时应用此配置
    if (isServer) {
      // 将 'ssh2' 标记为外部模块，防止 Webpack 尝试打包其原生依赖
      config.externals.push('ssh2');
    }
    return config;
  },
  eslint: {
    ignoreDuringBuilds: true,
  },
  typescript: {
    ignoreBuildErrors: true,
  },
  images: {
    unoptimized: true,
  },
};

export default nextConfig;
