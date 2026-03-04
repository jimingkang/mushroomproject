/** @type {import('next').NextConfig} */
const nextConfig = {
  typescript: {
    ignoreBuildErrors: true,
  },
  images: {
    unoptimized: true,
  },
  async rewrites() {
    return [
      {
        source: '/api/gripper/:path*',
        destination: 'http://172.23.248.37:5002/api/gripper/:path*',
      },
    ];
  },
}

export default nextConfig
