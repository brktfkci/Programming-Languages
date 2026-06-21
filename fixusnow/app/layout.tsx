import type { Metadata } from "next";
import { Geist } from "next/font/google";
import "./globals.css";
import { SessionProvider } from "next-auth/react";
import { Navbar } from "@/components/navbar";

const geist = Geist({ variable: "--font-geist-sans", subsets: ["latin"] });

export const metadata: Metadata = {
  title: "FixUsNow – Find Local Service Providers",
  description:
    "Post any home or local service task and get matched with trusted providers in your area.",
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en" className={`${geist.variable} h-full antialiased`}>
      <body className="min-h-full flex flex-col bg-gray-50">
        <SessionProvider>
          <Navbar />
          <main className="flex-1">{children}</main>
          <footer className="bg-white border-t border-gray-200 py-8 mt-16">
            <div className="max-w-7xl mx-auto px-4 text-center text-gray-500 text-sm">
              © {new Date().getFullYear()} FixUsNow. All rights reserved.
            </div>
          </footer>
        </SessionProvider>
      </body>
    </html>
  );
}
